#ifndef GOOSEKA_CONTROL_H
#define GOOSEKA_CONTROL_H

#include <stdint.h>
#include <Arduino.h>

/** Control constants */
#define KP_LINEAR 8. // FIXME: bulebule specs
#define KD_LINEAR 16. // FIXME: bulebule specs

#define KP_ANGULAR .05 // FIXME: bulebule specs .5
#define KD_ANGULAR 0.0 // FIXME: bulebule specs 1.0

#define MAX_ANGULAR_VELOCITY 20 // x radsps
#define ZERO_ANGULAR_DUTY 128
#define DRIVER_PWM_PERIOD 255
#define SCALE_ANGULAR_DUTY 0.0078125 * MAX_ANGULAR_VELOCITY

struct control_constants {
  float kp_linear;
  float kd_linear;
  float kp_angular;
  float kd_angular;
};


int32_t voltage_to_motor_pwm(float voltage, int32_t pwm_min, int32_t pwm_max);
void set_control_constants(struct control_constants value); // Used by mmlib (command) 
struct control_constants get_control_constants(void);

void reset_angular_control(void);
float get_angular_control(float angular_target_velocity,
                     float angular_meas_velocity);
float translate_duty_to_angular_velocity(uint8_t angular_duty);
float translate_angular_error_to_duty(float angular_control); 
float translate_manual_control_to_duty(uint8_t angular_duty, float linear_value);


#endif /* GOOSEKA_CONTROL_H */


