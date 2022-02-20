#ifndef GOOSEKA_MPU_H
#define GOOSEKA_MPU_H

#include<Wire.h>

uint8_t mpu_who_am_i(void);
void setup_mpu(void);
void gyro_z_calibration(void);
void update_gyro_readings(void);
float get_gyro_z_degrees(void);
int16_t get_gyro_z_raw(void);
float get_gyro_z_radps(void);
float get_gyro_z_dps(void);

#endif /* GOOSEKA_MPU_H */
