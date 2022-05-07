#ifndef GOOSEKA_DEFS_H
#define GOOSEKA_DEFS_H

// MOTOR PARAMETERS
#define MOTOR_POLES 14  

// ESC PINS
#define LEFT_PWM_PIN 25
#define LEFT_TELEMETRY_READ_PIN 35
#define LEFT_TELEMETRY_UNUSED_PIN 17
#define LEFT_TELEMETRY_SERIAL_BAUDS 115200

#define RIGHT_PWM_PIN 23
#define RIGHT_TELEMETRY_READ_PIN 34
#define RIGHT_TELEMETRY_UNUSED_PIN 13
#define RIGHT_TELEMETRY_SERIAL_BAUDS 115200

// GYRO UPDATE TIME
#define GYRO_UPDATE_TIME 50

// ESC PWM CONFIGURATION
#define PWM_MIN 1040
#define PWM_MAX 1960

// TELEMETRY SERIAL BUFFER
#define SERIAL_BUFFER_SIZE 10

// LORA TRANSCEIVER PINS
#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_SS 18
#define LORA_RST 14
#define LORA_DIO0 26

// LORA FREQUENCY BAND
// 433E6 for Asia
// 866E6 for Europe
// 915E6 for North America
#define LORA_BAND 866E6
#define LORA_SPREADING_FACTOR 7
#define LORA_BANDWIDTH 250e3
#define LORA_CODING_RATE 5
#define LORA_PREAMBLE_LENGTH 8
#define LORA_TX_POWER 20

// Minimum eRPM to consider that the car is running
#define MIN_RPM_START 3

// LORA SYNCWORD
#define LORA_SYNCWORD 0xCA

// IF NOTHING RECEIVED FOR MORE THAN 5s, STOP MOTORS
#define RADIO_IDLE_TIMEOUT 5000L

// LORA SENDER SLOW DOWN 
// (do not send more than one msg every LORA_SLOWDOWN ms)
#define LORA_SLOWDOWN 100L

// INTER CPU MESG QUEUE SIZE
#define QUEUE_SIZE 1

// USB SERIAL SPEED
#define SERIAL_BAUDRATE 115200

// MAGIC NUMBER TO CHECK FOR USB ERRORS
#define MAGIC_NUMBER 0xCA

// Number of samples for MPPT
#define NUM_SAMPLES_MPPT 10

// Number of ms between lineal pid executions
#define MS_LINEAR_CONTROL 150L

// Number of ms between MPPT meas

#define MS_MPPT_MEAS 15L

// Do not allow MPPT go down this value (unless linear target is lower than this value)
#define LINEAR_MPPT_MIN 30.0

// Threshold value in order to apply the mppt
#define LINEAR_MPPT_THRESHOLD 30
#define LINEAR_MPPT_STEP 5

// Startup time (during this time MPPT is not applied)
#define MS_STARTUP_TIME 0L  //10000L 

#define MANUAL_STEERING 1 // Use a value bigger than 0 to use manual steering

#endif /* GOOSEKA_DEFS_H */
