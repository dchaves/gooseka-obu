#include "gooseka_control.h"

uint32_t last_mppt_update_millis;
uint32_t last_linear_update_millis;
float last_linear_value;
float linear_value;
uint8_t mppt_idx;
uint32_t last_linear_mppt;
uint32_t linear_mppt;

// Variables needed to calculate MR
uint32_t last_voltage_mppt;
uint32_t last_current_mppt;
uint32_t voltage_mppt;
uint32_t current_mppt;

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
  last_voltage_mppt = 0;
  last_current_mppt = 0;
  voltage_mppt = 0;
  current_mppt = 0;
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


uint8_t get_step_with_mr(float mr, uint8_t direction) {

  // Calculate step with mr  
  uint8_t step = 0.0;
    
  // Reverse mr when going down
  if (direction == 0) {
    mr *= -1.0;
  }  

  if ((mr >= LINEAR_MPPT_MINSTEP) && (mr <= LINEAR_MPPT_MAXSTEP)) {
    step = (uint8_t) mr;
  }
  else if (mr > LINEAR_MPPT_MAXSTEP) {
    step = LINEAR_MPPT_MAXSTEP;
  }
  else {
    step = LINEAR_MPPT_MINSTEP;
  }

  return step;
}

float calculate_mr(sample_t* samples) {
    voltage_mppt = 0;
    current_mppt = 0;

    for (int i = 0; i < NUM_SAMPLES_MPPT; i++) {
      current_mppt += samples[i].current;
      voltage_mppt += samples[i].voltage;
    }

    float dev_voltage = (float) (voltage_mppt - last_voltage_mppt);
    float dev_current = (float) (current_mppt - last_current_mppt);

    float mr = 0.0;
    
    if (dev_voltage != 0) {
      mr = (voltage_mppt * dev_current) / (dev_voltage * current_mppt);

      }
    else {
      mr = 0.0;
    }

    last_voltage_mppt = voltage_mppt;
    last_current_mppt = current_mppt;

    return mr;
}

uint8_t calculate_mppt_duty(uint8_t target_duty, sample_t* samples) {
  if (time_since(last_linear_update_millis) > MS_LINEAR_CONTROL) {
    last_linear_mppt = linear_mppt;
    linear_mppt = 0;

    for (int i = 0; i < NUM_SAMPLES_MPPT; i++) {
      linear_mppt += samples[i].current * samples[i].voltage;

    }

    float mr = calculate_mr(samples);
    uint8_t step = 0;
    
    if (linear_value >= last_linear_value) {   // Increased the velocity in the period
      last_linear_value = linear_value;        // storing last linear value before modifying linear value
      if (linear_mppt >= last_linear_mppt) {   // MPPT has increased in the period
        step = get_step_with_mr(mr, MPPT_DIRECTION_UP);
        linear_value += step;
      } else {                                 // MPPT has decreased in the period
        step = get_step_with_mr(mr, MPPT_DIRECTION_DOWN);
        linear_value -= step;
      }
    } else {                                   // Decreased the velocity in the period
        last_linear_value = linear_value;      // storing last linear value before modifying linear value
        if (linear_mppt >= last_linear_mppt) { // MPPT has increased in the period
          step = get_step_with_mr(mr, MPPT_DIRECTION_DOWN);
          linear_value -= step;
        } else {                               // MPPT has decreased in the period
          step = get_step_with_mr(mr, MPPT_DIRECTION_UP);
          linear_value += step;
        }
    }

    if (target_duty > LINEAR_MPPT_MIN) {
      linear_value = constrain(linear_value, LINEAR_MPPT_MIN, target_duty);
    }
    else {
      linear_value = constrain(linear_value, 0.0, target_duty);
    }
    last_linear_update_millis = millis();
  }
  
  return (uint8_t)(constrain(linear_value, 0.0, 255.0));
}
