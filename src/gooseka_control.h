#ifndef GOOSEKA_CONTROL_H
#define GOOSEKA_CONTROL_H

#include <stdint.h>
#include <Arduino.h>

/** Control constants */
#define KP_LINEAR 8. // FIXME: bulebule specs
#define KD_LINEAR 16. // FIXME: bulebule specs

#define KP_ANGULAR .05 // FIXME: bulebule specs
#define KD_ANGULAR 1. // FIXME: bulebule specs

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

#endif /* GOOSEKA_CONTROL_H */


