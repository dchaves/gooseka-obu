#include "gooseka_control.h"

static volatile float angular_error;
static volatile float last_angular_error;

static struct control_constants control = {
                                           .kp_linear = KP_LINEAR,
                                           .kd_linear = KD_LINEAR,
                                           .kp_angular = KP_ANGULAR,
                                           .kd_angular = KD_ANGULAR};


struct control_constants get_control_constants(void)
{
  return control;
}


/*
  @brief transform voltage to pwm duty

 */
int32_t voltage_to_motor_pwm(float voltage, int32_t pwm_min, int32_t pwm_max)
{
  return constrain(voltage * (pwm_max - pwm_min) + pwm_min, pwm_min, pwm_max);
}

void set_control_constants(struct control_constants value)
{
  control = value;
}

/*
 * @brief reset internal pid control
 */
void reset_angular_control(void)
{
  angular_error = 0.0;
  last_angular_error = 0.0;
}

/*
 * @brief update angular contribution to pwm
 */
float angular_control(float angular_target_velocity,
                     float angular_meas_velocity)
{
  float angular_voltage;
  
   angular_error += angular_target_velocity - angular_meas_velocity;

   control = get_control_constants();

   angular_voltage =  control.kp_angular * angular_error +
     control.kd_angular * (angular_error - last_angular_error);

   return angular_voltage;
   
}

