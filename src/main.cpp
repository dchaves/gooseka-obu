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

float update_angular_control(float target_angular_vel) {

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
    // variable to store last angular duty and resets pid error if a new angular duty arrives
    uint8_t last_angular_duty = 0;
    ESC_telemetry_t telemetry, telemetry_controller;
    
    float control_target_left = 0.0;
    float control_target_right = 0.0;
    uint8_t pwm_left = 0;
    uint8_t pwm_right = 0;
    float angular_control_pid = 0.0;

    uint32_t last_mppt_update_millis = 0;
    uint32_t last_linear_update_millis = 0;
    uint32_t time_since_last_stop = 0;
    uint32_t mppt_samples_current[NUM_SAMPLES_MPPT];
    uint32_t mppt_samples_voltage[NUM_SAMPLES_MPPT];
    uint8_t mppt_idx = 0;
    uint32_t last_linear_mppt = 0;
    uint32_t linear_mppt = 0;
    float last_mppt_value = 0.0;
    float linear_value = 0.0;
    float last_linear_value = 0.0;
    
    
    memset(&telemetry,0,sizeof(ESC_telemetry_t));
    memset(&telemetry_controller,0, sizeof(ESC_telemetry_t));
    memset(&control,0,sizeof(ESC_control_t));


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


        // Read Telemetry packet
        xQueueReceive(telemetry_queue, &telemetry, 0); // EMPTY TELEMETRY QUEUE

        // Check mppt meas (current and voltage)
        if (time_since(last_mppt_update_millis) > MS_MPPT_MEAS) {
          mppt_samples_current[mppt_idx] = telemetry.left.current + telemetry.right.current;
          mppt_samples_voltage[mppt_idx] = (telemetry.left.voltage + telemetry.right.voltage) >> 1;
          mppt_idx++;
          mppt_idx = mppt_idx % NUM_SAMPLES_MPPT;
          last_mppt_update_millis = millis();
        }
        
        // Resetting angular pid if a different order arrives
        if (last_angular_duty  != control.angular.duty)
          {
            reset_angular_control();
            last_angular_duty = control.angular.duty;
          }
        
        // TODO (linear error is not calculated yet
        // Linear voltage should be a function of the linear error (not calculated yet)
        float linear_target = control.linear.duty;

        if ((time_since(last_linear_update_millis) > MS_LINEAR_CONTROL) && (time_since(time_since_last_stop) > MS_STARTUP_TIME)) {
          last_linear_mppt = linear_mppt;
          
          linear_mppt = 0;          
          for (int i = 0; i < NUM_SAMPLES_MPPT; i++) {
            linear_mppt += mppt_samples_current[i] * mppt_samples_voltage[i];
          }

          // Apply control only with a minimum velocity
          if (linear_target > LINEAR_MPPT_THRESHOLD) {

            // Increased the velocity in the period
            if (linear_value >= last_linear_value) {
              // storing last linear value before modifying linear value
              last_linear_value = linear_value;

              // MPPT has increased in the period
              if (linear_mppt >= last_linear_mppt) {
                linear_value += LINEAR_MPPT_STEP;
              }
              else {
                linear_value -= LINEAR_MPPT_STEP;
              }
            }
            // Decreased the velocity in the period
            else {

              // storing last linear value before modifying linear value
              last_linear_value = linear_value;
              
              // MPPT has increased in the period
              if (linear_mppt >= last_linear_mppt) {
                linear_value -= LINEAR_MPPT_STEP;
              }
              else {
                linear_value += LINEAR_MPPT_STEP;
              }
            }

            if (linear_target > LINEAR_MPPT_MIN) {
              linear_value = constrain(linear_value, LINEAR_MPPT_MIN, linear_target);
            }
            else {
              linear_value = constrain(linear_value, 0.0, linear_target);
            }
          }
          else
          {
            last_linear_value = linear_value;
            linear_value = linear_target;
          }
          
          last_linear_update_millis = millis();
        }
        
        // We scale to a maximum allowed angular velocity
        float angular_target = translate_duty_to_angular_velocity(control.angular.duty);

        // Apply angular control only if erpm is highter than a certain value

        if (time_since(last_angular_update_millis) > GYRO_UPDATE_TIME) {
          angular_control_pid = update_angular_control(angular_target);
          last_angular_update_millis = millis();
        }

        // Calculate always because the PID needs to be updated at concrete intervals !! but apply only if certain erpms are achieved
        
        if (((telemetry.left.erpm + telemetry.right.erpm)/2) < MIN_RPM_START) {
          angular_control_pid = 0;
        }
               
        if (linear_target > 0) {
          float angular_pid_duty = 0.0;

          // Manual control
          if (MANUAL_STEERING > 0) {
            angular_pid_duty = translate_manual_control_to_duty(control.angular.duty, linear_value);
          }
          else {
            angular_pid_duty = translate_angular_error_to_duty(angular_control_pid);
          }
          control_target_left = linear_value + angular_pid_duty; 
          control_target_right = linear_value - angular_pid_duty;

          pwm_left = (uint8_t) constrain(control_target_left,0.0,255.0);
          pwm_right = (uint8_t) constrain(control_target_right,0.0,255.0);  
        }
        else {
            pwm_left = 0;
            pwm_right = 0;
            time_since_last_stop = millis();
        }        
        
        LEFT_ESC_servo.write(map(pwm_left,0,255,0,180));
        RIGHT_ESC_servo.write(map(pwm_right,0,255,0,180));

        // Send packet 
        if(time_since(last_sent_millis) > LORA_SLOWDOWN) {
            // Send enqueued msgs
            last_sent_millis = millis();

            memcpy(&telemetry_controller, &telemetry, sizeof(ESC_telemetry_t));            

            // Modify telemetry that is send to the controller
            // Send duty
            telemetry_controller.left.duty = pwm_left;
            telemetry_controller.right.duty = pwm_right;

            // Send angular velocity in temperature
            float meas_angular_vel = get_gyro_z_radps();

            if (meas_angular_vel > 0) {

              telemetry_controller.left.temperature = (uint16_t) meas_angular_vel * 1000;
              telemetry_controller.right.temperature = 0;
            }
            else {
              telemetry_controller.left.temperature = 0;
              telemetry_controller.right.temperature = (uint16_t) (meas_angular_vel * -1000);
            }
            
            send_via_radio((uint8_t *)&telemetry_controller, sizeof(ESC_telemetry_t));
            //digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            DEBUG_TELEMETRY(&Serial, &telemetry_controller);
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
