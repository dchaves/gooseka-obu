#ifndef GOOSEKA_CONTROL_H
#define GOOSEKA_CONTROL_H
#include <Arduino.h>
#include "gooseka_structs.h"
#include "gooseka_helpers.h"

// Number of samples for MPPT
#define NUM_SAMPLES_MPPT 10
#define NUM_SAMPLES_MPPT_SETUP 50
#define ZERO_ANGULAR_DUTY 128
// Number of ms between lineal pid executions
#define MS_LINEAR_CONTROL 150L
// Number of ms between MPPT meas
#define MS_MPPT_MEAS 15L
// Do not allow MPPT go down this value (unless linear target is lower than this value)
#define LINEAR_MPPT_MIN 30.0
#define LINEAR_MPPT_STEP 5

// Voltage ratio from the reference to consider we are in the normal state
#define MPPT_VOLTAGE_NORMAL_RATIO 0.90
#define MPPT_VOLTAGE_DANGER_RATIO 0.8
#define MPPT_VOLTAGE_MINREF 1200 // 1200 centivolts
#define LINEAR_MPPT_MAX_STEP 15
#define LINEAR_MPPT_MIN_STEP 2

typedef struct __attribute__((packed)) {
    uint32_t voltage;
    uint32_t current;
} sample_t;

float angular_to_linear(uint8_t angular_duty, float linear_value);
void init_mppt();
void update_mppt_measurements(ESC_telemetry_t* telemetry, sample_t* samples);
void update_mppt_setup_measurements(ESC_telemetry_t* telemetry, sample_t*samples);
uint8_t calculate_mppt_duty(uint8_t target_duty, sample_t* samples, sample_t* setup_samples);

#endif /* GOOSEKA_CONTROL_H */


