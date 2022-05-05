#include "gooseka_control.h"

static volatile float angular_error = 0.0;
static volatile float last_angular_error = 0.0;

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
  @brief transform angular duty to angular velocity
 */
float translate_duty_to_angular_velocity(uint8_t angular_duty)
{
  float angular_velocity = (angular_duty - ZERO_ANGULAR_DUTY) * SCALE_ANGULAR_DUTY;

  return angular_velocity;
}


/*
  @brief transform angular error to duty
*/
float  translate_angular_error_to_duty(float angular_control)
{
  float angular_duty = (angular_control/(MAX_ANGULAR_VELOCITY * 2)) * DRIVER_PWM_PERIOD;

  return angular_duty;
}


/*
  @brief transform angular duty to duty
*/
float  translate_manual_control_to_duty(uint8_t angular_duty, float linear_value)
{
  float applied_angular_duty = ((float) ((float) angular_duty - ZERO_ANGULAR_DUTY))/ZERO_ANGULAR_DUTY;

  applied_angular_duty *= linear_value;
  
  return applied_angular_duty;
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
float get_angular_control(float angular_target_velocity,
                     float angular_meas_velocity)
{
  float angular_control;
  
  last_angular_error = angular_error;
   angular_error += angular_target_velocity - angular_meas_velocity;

   control = get_control_constants();

   angular_control =  (control.kp_angular * angular_error) +
     (control.kd_angular * (angular_error - last_angular_error));

   return angular_control;
   
}

