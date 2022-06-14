#include <Arduino.h>
#include <HardwareSerial.h>
#include <ESP32Servo.h>
#include "gooseka_helpers.h"
#include "gooseka_control.h"
#include "gooseka_lora.h"

QueueHandle_t telemetry_queue;

// CPU #1
// 0. Read incoming message
// 1. Set LEFT & RIGHT ESC duty cycle
void radio_receive_task(void* param) {
    uint8_t radio_buffer[sizeof(ESC_control_t)];
    ESC_control_t control;
    ESC_telemetry_t telemetry;
    sample_t samples[NUM_SAMPLES_MPPT];
    uint32_t last_received_millis;
    uint32_t last_sent_millis;
    Servo LEFT_ESC_servo;
    Servo RIGHT_ESC_servo;
    
    uint8_t pwm_left = 0;
    uint8_t pwm_right = 0;

    bool new_telemetry = false;    
    
    memset(&telemetry,0,sizeof(ESC_telemetry_t));
    memset(&control,0,sizeof(ESC_control_t));
    memset(samples,0, NUM_SAMPLES_MPPT*sizeof(sample_t));
    init_mppt();

    last_sent_millis = millis();
    last_received_millis = 0;
    
    // Configure servos
    LEFT_ESC_servo.attach(LEFT_PWM_PIN, PWM_MIN, PWM_MAX);
    RIGHT_ESC_servo.attach(RIGHT_PWM_PIN, PWM_MIN, PWM_MAX);

    while(1) {
        int packetSize = receive_radio_packet(radio_buffer, sizeof(ESC_control_t));
        // DEBUG_PRINTLN(packetSize);
        if (packetSize == sizeof(ESC_control_t)) {
          if(((ESC_control_t*)radio_buffer)->magic_number == MAGIC_NUMBER) { // IF MAGIC NUMBER IS OK, USE RECEIVED CONTROL
            memcpy(&control, radio_buffer, sizeof(ESC_control_t));           // IF NOT, KEEP PREVIOUS CONTROL
            last_received_millis = millis();
                
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
            control.angular.duty = 16; // 128 is translated to 0 radsps
        }

        // Read Telemetry packet & EMPTY TELEMETRY QUEUE
        if(xQueueReceive(telemetry_queue, &telemetry, 0)) { 
          new_telemetry = true;
          float current_factor = 1.0 - ((control.angular.duty - 128.0) / 128.0);
          telemetry.right.current = (uint16_t)(telemetry.left.current * current_factor);
          telemetry.right.voltage = telemetry.left.voltage;
          telemetry.right.power = telemetry.left.power;
          telemetry.right.erpm = telemetry.left.erpm;
          telemetry.right.temperature = telemetry.left.temperature;
          telemetry.right.timestamp = telemetry.left.timestamp;
          telemetry.right.duty = telemetry.left.duty;
        } else {
          new_telemetry = false;
        }

        // Check mppt meas (current and voltage)
        if(new_telemetry){ // ONLY UPDATE MEASUREMENTS IF TELEMETRY IS FRESH
          update_mppt_measurements(&telemetry, samples);
        }
        uint8_t mppt_linear_duty = calculate_mppt_duty(control.linear.duty, samples);
        
        // Manual control
        float angular_pid_duty = angular_to_linear(control.angular.duty, mppt_linear_duty);
        float control_target_left = mppt_linear_duty + angular_pid_duty; 
        float control_target_right = mppt_linear_duty - angular_pid_duty;

        pwm_left = (uint8_t) constrain(control_target_left,0.0,255.0);
        pwm_right = (uint8_t) constrain(control_target_right,0.0,255.0);

        if(control.linear.duty == 0) { // IF LINEAR DUTY IS 0, FORCE STOP
            pwm_left = 0;
            pwm_right = 0;
        }
                
        LEFT_ESC_servo.write(map(pwm_left,0,255,0,180));
        RIGHT_ESC_servo.write(map(pwm_right,0,255,0,180));

        // Send packet 
        if((time_since(last_sent_millis) > LORA_SLOWDOWN) && new_telemetry) {
            // Send enqueued msgs
            last_sent_millis = millis();          

            // Modify telemetry that is sent to the controller
            // Send duty
            telemetry.left.duty = pwm_left;
            telemetry.right.duty = pwm_right;
            
            send_via_radio((uint8_t *)&telemetry, sizeof(ESC_telemetry_t));
            //digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            DEBUG_TELEMETRY(&Serial, &telemetry);
        }

        // FIXME: maybe increase delay??
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
        if(LEFT_ESC_serial.available()) {
            // DEBUG_PRINTLN("LEFT AVAILABLE");
            LEFT_telemetry_complete = false;
            if(read_telemetry(&LEFT_ESC_serial, &LEFT_serial_buffer, &(telemetry.left))) {
                LEFT_telemetry_complete = true;
                // DEBUG_PRINTLN("LEFT TELEMETRY");
            }
        }

        if(RIGHT_ESC_serial.available()) {
            // DEBUG_PRINTLN("RIGHT AVAILABLE");
            RIGHT_telemetry_complete = false;    
            if(read_telemetry(&RIGHT_ESC_serial, &RIGHT_serial_buffer, &(telemetry.right))) {
                RIGHT_telemetry_complete = true;
                // DEBUG_PRINTLN("RIGHT TELEMETRY");
            }
        }

        if(LEFT_telemetry_complete /*&& RIGHT_telemetry_complete*/) {
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
