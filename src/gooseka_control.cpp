#include "gooseka_control.h"

uint32_t last_mppt_update_millis;
uint32_t last_linear_update_millis;
float last_linear_value;
float linear_value;
uint8_t mppt_idx;
uint32_t last_linear_mppt;
uint32_t linear_mppt;

uint8_t mppt_setup_idx;
float voltage_mppt_setup;


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

  // Initial setup
  mppt_setup_idx = 0;
  voltage_mppt_setup = 0;
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

float get_mppt_voltage(sample_t* samples, uint8_t num_samples) {

  float ref_voltage = 0.0;
  uint32_t num_samples = 0;
  
  for (int i = 0; i < num_samples; i++){
    if (samples[i].voltage > 0) {
      ref_voltage += samples[i].voltage;
    }    
  }

  if (num_samples > 0) {
    ref_voltage /= num_samples;
  }
  else {
    ref_voltage = 0;
  }    
}

void update_mppt_setup_measurements(ESC_telemetry_t* telemetry, sample_t*samples) {
   samples[mppt_setup_idx].current = telemetry->left.current + telemetry->right.current;
   samples[mppt_setup_idx].voltage = (telemetry->left.voltage + telemetry->right.voltage) >> 1;
   mppt_setup_idx++;
   mppt_setup_idx = mppt_setup_idx % NUM_SAMPLES_MPPT_SETUP;
}

uint8_t calculate_mppt_duty(uint8_t target_duty, sample_t* samples, sample_t* setup_samples) {

  
  if (time_since(last_linear_update_millis) > MS_LINEAR_CONTROL) {
    last_linear_mppt = linear_mppt;
    linear_mppt = 0;          
    for (int i = 0; i < NUM_SAMPLES_MPPT; i++) {
      linear_mppt += samples[i].current * samples[i].voltage;
    }

    float ref_voltage = get_mppt_voltage(samples, NUM_SAMPLES_MPPT_SETUP);
    float current_voltage = get_mppt_voltage(samples, NUM_SAMPLES_MPPT);

    uint32_t up_step = LINEAR_MPPT_STEP;
    uint32_t down_step = LINEAR_MPPT_STEP;
    
    if ((current_voltage > 0) && (ref_voltage > 0)) {
      // Only apply this if ref_voltage is higher than a certain threshold
      if (ref_voltage > MPPT_VOLTAGE_MINREF) {
        if (current_voltage > MPPT_VOLTAGE_NORMAL_RATIO * ref_voltage) {
          up_step = LINEAR_MPPT_MAX_STEP;
          down_step = LINEAR_MPPT_MIN_STEP;
        } else if (current_voltage < MPPT_VOLTAGE_DANGER_RATIO * ref_voltage) {
          up_step = LINEAR_MPPT_MIN_STEP;
          down_step = LINEAR_MPPT_MAX_STEP;
        }
      }
    }
      
    
    if (linear_value >= last_linear_value) {   // Increased the velocity in the period
      last_linear_value = linear_value;        // storing last linear value before modifying linear value
      if (linear_mppt >= last_linear_mppt) {   // MPPT has increased in the period
        linear_value += up_step;
      } else {                                 // MPPT has decreased in the period
        linear_value -= down_step;
      }
    } else {                                   // Decreased the velocity in the period
        last_linear_value = linear_value;      // storing last linear value before modifying linear value
        if (linear_mppt >= last_linear_mppt) { // MPPT has increased in the period
          linear_value -= down_step;
        } else {                               // MPPT has decreased in the period
          linear_value += up_step;
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
