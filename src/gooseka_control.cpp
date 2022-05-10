#include "gooseka_control.h"

uint32_t last_mppt_update_millis;
uint32_t last_linear_update_millis;
float last_linear_value;
float linear_value;
uint8_t mppt_idx;
uint32_t last_linear_mppt;
uint32_t linear_mppt;

float angular_to_linear(uint8_t angular_duty, float linear_value) {
  float applied_angular_duty = ((float) ((float) angular_duty - ZERO_ANGULAR_DUTY))/ZERO_ANGULAR_DUTY;
  applied_angular_duty *= linear_value;
  return applied_angular_duty;
}

void init_mppt() {
  last_mppt_update_millis = 0;
  last_linear_update_millis = 0;
  last_linear_value = 0.0;
  linear_value = 0;
  mppt_idx = 0;
  last_linear_mppt = 0;
  linear_mppt = 0;
}

void update_mppt_measurements(ESC_telemetry_t* telemetry, sample_t* samples) {
  if (time_since(last_mppt_update_millis) > MS_MPPT_MEAS) {
    samples[mppt_idx].current = telemetry->left.current + telemetry->right.current;
    samples[mppt_idx].voltage = (telemetry->left.voltage + telemetry->right.voltage) >> 1;
    mppt_idx++;
    mppt_idx = mppt_idx % NUM_SAMPLES_MPPT;
    last_mppt_update_millis = millis();
  }
}

uint8_t calculate_mppt_duty(uint8_t target_duty, sample_t* samples) {
  if (time_since(last_linear_update_millis) > MS_LINEAR_CONTROL) {
    last_linear_mppt = linear_mppt;
    linear_mppt = 0;          
    for (int i = 0; i < NUM_SAMPLES_MPPT; i++) {
      linear_mppt += samples[i].current * samples[i].voltage;
    }

    // storing last linear value before modifying linear value
    last_linear_value = linear_value;
    
    if (linear_value >= last_linear_value) {   // Increased the velocity in the period
      if (linear_mppt >= last_linear_mppt) {   // MPPT has increased in the period
        linear_value += LINEAR_MPPT_STEP;
      } else {
        linear_value -= LINEAR_MPPT_STEP;
      }
    } else {                                   // Decreased the velocity in the period
        if (linear_mppt >= last_linear_mppt) { // MPPT has increased in the period
          linear_value -= LINEAR_MPPT_STEP;
        }
        else {
          linear_value += LINEAR_MPPT_STEP;
        }
    }

    if (target_duty > LINEAR_MPPT_MIN) {
      linear_value = constrain(linear_value, LINEAR_MPPT_MIN, linear_value);
    }
    else {
      linear_value = constrain(linear_value, 0.0, linear_value);
    }
    last_linear_update_millis = millis();
  }
  
  return (uint8_t)(constrain(linear_value, 0.0, 255));
}