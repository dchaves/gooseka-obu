#include <Arduino.h>
#include <HardwareSerial.h>
#include <ESP32Servo.h>
#include <SPI.h>
#include <LoRa.h>


#include "gooseka_helpers.h"
#include "gooseka_structs.h"
#include "gooseka_defs.h"
#include "gooseka_mpu.h"
#include "gooseka_control.h"


QueueHandle_t control_queue;
QueueHandle_t telemetry_queue;



void init_radio() {
    // Set SPI LoRa pins
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);

    // Setup LoRa transceiver module
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
    if(!LoRa.begin(LORA_BAND)) {
        DEBUG_PRINTLN("LoRa init error.");
        while(1);
    }
    LoRa.setCodingRate4(LORA_CODING_RATE);
    LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
    LoRa.setSignalBandwidth(LORA_BANDWIDTH);
    LoRa.setTxPower(LORA_TX_POWER);
    LoRa.setSyncWord(LORA_SYNCWORD);
}

void send_via_radio(uint8_t* payload, size_t size) {
    LoRa.beginPacket();
    LoRa.write(payload, size); 
    if(LoRa.endPacket() == 1) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
}

int receive_radio_packet(uint8_t* buffer, int size) {
    uint8_t index = 0;

    int packetSize = LoRa.parsePacket();
    if (packetSize == size) {
        while (LoRa.available() && index < size) {
            buffer[index] = LoRa.read();
            index++;
        }
    }

    return packetSize;
}

uint32_t time_since(uint32_t reference_time) {
    return millis() - reference_time;
}

float update_angular_control(uint32_t* last_update_millis, float target_angular_vel) {
    // Do this once every GYRO_UPDATE_TIME ms
    if (time_since(*last_update_millis) < GYRO_UPDATE_TIME) {
        return 0.0; // TODO check if it is better to return last value or 0
    }
    *last_update_millis = millis();
    update_gyro_readings();
    DEBUG_PRINT("DPS: ");
    DEBUG_PRINTLN(get_gyro_z_dps());

    // Measure angular velocity
    float meas_angular_vel = get_gyro_z_radps();
    float angular_control = get_angular_control(target_angular_vel, meas_angular_vel);

    return angular_control;
}

// CPU #1
// 0. Read incoming message
// 1. Set LEFT & RIGHT ESC duty cycle
void radio_receive_task(void* param) {
    uint8_t radio_buffer[sizeof(ESC_control_t)];
    // uint8_t index;
    ESC_control_t control;
    uint32_t last_received_millis;
    Servo LEFT_ESC_servo;
    Servo RIGHT_ESC_servo;
    uint32_t last_sent_millis;
    uint32_t last_angular_update_millis;
    ESC_telemetry_t telemetry;
    //TODO: Should add the linear control in the future
    MPU_angular_control_t angular_control_msg;
    
    float angular_control_left = 0.0;
    float angular_control_right = 0.0;
    uint8_t pwm_left = 0;
    uint8_t pwm_right = 0;

    memset(&telemetry,0,sizeof(ESC_telemetry_t));
    memset(&control,0,sizeof(ESC_control_t));
    memset(&angular_control_msg,0,sizeof(MPU_angular_control_t));

    last_sent_millis = millis();
    last_received_millis = 0;
    last_angular_update_millis = 0;
    
    // Configure servos
    LEFT_ESC_servo.attach(LEFT_PWM_PIN, PWM_MIN, PWM_MAX);
    RIGHT_ESC_servo.attach(RIGHT_PWM_PIN, PWM_MIN, PWM_MAX);

    while(1) {
        int packetSize = receive_radio_packet(radio_buffer, sizeof(ESC_control_t));
        // DEBUG_PRINTLN(packetSize);
        if (packetSize == sizeof(ESC_control_t)) {
            memcpy(&control, radio_buffer, sizeof(ESC_control_t));
            last_received_millis = millis();
            if(control.magic_number == MAGIC_NUMBER) {
                xQueueSend(control_queue, &control, 0);
                
                DEBUG_PRINT("Received commands: ");
                DEBUG_PRINT(control.linear.duty);
                DEBUG_PRINT(",");
                DEBUG_PRINT(control.angular.duty);
                DEBUG_PRINT(",");
                DEBUG_PRINTLN(LoRa.packetRssi());
            }
        } else if(time_since(last_received_millis) > RADIO_IDLE_TIMEOUT) {
            DEBUG_PRINTLN("OUT OF RANGE");
            last_received_millis = millis();
            control.linear.duty = 0;
            control.angular.duty = 128; // 128 is translated to 0 radsps
        }

        // TODO (linear error is not calculated yet
        // Linear voltage should be a function of the linear error (not calculated yet)
        float linear_target = control.linear.duty;
        float angular_target = translate_angular_velocity(control.angular.duty);
        
        float angular_control_pid = update_angular_control(&last_angular_update_millis, angular_target);
        
        if (linear_target > 0) {        
          angular_control_left = linear_target + angular_control_pid;
          angular_control_right = linear_target - angular_control_pid;
          

          pwm_left = voltage_to_motor_pwm(angular_control_left, 0, 255);
          pwm_right = voltage_to_motor_pwm(angular_control_right, 0, 255);

          
        }
        else
          {
            pwm_left = 0;
            pwm_right = 0;
          }        
        
        LEFT_ESC_servo.write(map(pwm_left,0,255,0,180));
        RIGHT_ESC_servo.write(map(pwm_right,0,255,0,180));

        if(time_since(last_sent_millis) > LORA_SLOWDOWN) {
            // Send enqueued msgs
            last_sent_millis = millis();
            xQueueReceive(telemetry_queue, &telemetry, 0);
            send_via_radio((uint8_t *)&telemetry, sizeof(ESC_telemetry_t));
            //digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            DEBUG_TELEMETRY(&Serial, &telemetry);
        }
        vTaskDelay(1); // Without this line watchdog resets the board
    }
    vTaskDelete(NULL);
}

// CPU #0
// 0. Set LEFT & RIGHT ESC PWM
// 1. Read LEFT ESC telemetry
// 2. Read RIGHT ESC telemetry
// 3. If both telemetries are complete, send message
void ESC_control_task(void* param) {
    ESC_control_t control;
    serial_buffer_t LEFT_serial_buffer;
    serial_buffer_t RIGHT_serial_buffer;
    ESC_telemetry_t telemetry;
    bool LEFT_telemetry_complete;
    bool RIGHT_telemetry_complete;
    HardwareSerial LEFT_ESC_serial(1);
    HardwareSerial RIGHT_ESC_serial(2);
    
    // Initialize structs and arrays
    LEFT_telemetry_complete = false;
    RIGHT_telemetry_complete = false;
    memset(&LEFT_serial_buffer,0,sizeof(serial_buffer_t));
    memset(&RIGHT_serial_buffer,0,sizeof(serial_buffer_t));
    memset(&telemetry,0,sizeof(ESC_telemetry_t));
    memset(&control,0,sizeof(ESC_control_t));

    // Telemetry serial lines
    LEFT_ESC_serial.begin(LEFT_TELEMETRY_SERIAL_BAUDS, SERIAL_8N1, LEFT_TELEMETRY_READ_PIN, LEFT_TELEMETRY_UNUSED_PIN);
    RIGHT_ESC_serial.begin(RIGHT_TELEMETRY_SERIAL_BAUDS, SERIAL_8N1, RIGHT_TELEMETRY_READ_PIN, RIGHT_TELEMETRY_UNUSED_PIN);

    // Empty Rx Serial of garbage telemetry
    while(LEFT_ESC_serial.available()) {
        LEFT_ESC_serial.read();
    }
    while(RIGHT_ESC_serial.available()) {
        RIGHT_ESC_serial.read();
    }

    while (1) {
        xQueueReceive(control_queue, &control, 0);
        
        if(LEFT_ESC_serial.available()) {
            // DEBUG_PRINTLN("LEFT AVAILABLE");
            if(read_telemetry(&LEFT_ESC_serial, &LEFT_serial_buffer, &(telemetry.left))) {
                telemetry.left.duty = control.linear.duty;
                LEFT_telemetry_complete = true;
                // DEBUG_PRINTLN("LEFT TELEMETRY");
            }
        }

        if(RIGHT_ESC_serial.available()) {
            // DEBUG_PRINTLN("RIGHT AVAILABLE");
            if(read_telemetry(&RIGHT_ESC_serial, &RIGHT_serial_buffer, &(telemetry.right))) {
                telemetry.right.duty = control.angular.duty;
                RIGHT_telemetry_complete = true;
                // DEBUG_PRINTLN("RIGHT TELEMETRY");
            }
        }

        if(LEFT_telemetry_complete && RIGHT_telemetry_complete) {
            LEFT_telemetry_complete = false;
            RIGHT_telemetry_complete = false;    
            // DEBUG_TELEMETRY(&Serial, &telemetry);
            xQueueSend(telemetry_queue, &telemetry, 0);
        }
        vTaskDelay(1); // Without this line watchdog resets the board
    }
    vTaskDelete(NULL);
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    // Console output
    DEBUG_BEGIN(SERIAL_BAUDRATE);

    // Init radio interface
    init_radio();

    // Setup MPU
    setup_mpu();

    // Init control msg queue
    control_queue = xQueueCreate(QUEUE_SIZE, sizeof(ESC_control_t));

    // Init telemetry msg queue
    telemetry_queue = xQueueCreate(QUEUE_SIZE, sizeof(ESC_telemetry_t));

    // Start ESC control task
    xTaskCreatePinnedToCore(ESC_control_task, "ESC_controller", 10000, NULL, 1, NULL, 0);
    // Start LoRa receiver task
    xTaskCreatePinnedToCore(radio_receive_task, "radio_receiver", 10000, NULL, 1, NULL, 1);
}

void loop() {
    delay(2147483647L); // Delay forever
}
