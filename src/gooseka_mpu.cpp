#include "gooseka_mpu.h"

#define BYTE 8
#define MPU_CAL_SAMPLE_NUM 100
#define MPU_AVERAGE_FACTOR 2
#define MPU_COMPLEMENT_2_FACTOR 2
#define MPU_CAL_SAMPLE_MS 1

#define MPU_SMPLRT_DIV 25

#define MPU_SMPLRT_DIV 25
#define MPU_CONFIG 26
#define MPU_GYRO_CONFIG 27
#define MPU_SIGNAL_PATH_RESET 104
#define MPU_PWR_MGMT_1 107
#define MPU_USER_CTRL 106
#define MPU_WHOAMI 117

#define MPU_GYRO_ZOUT_H 71
#define MPU_GYRO_ZOUT_L 72
#define MPU_Z_OFFS_USR_H 23
#define MPU_Z_OFFS_USR_L 24

#define MPU_MASK_H 0xFF00
#define MPU_MASK_L 0x00FF

#define MPU_GYRO_SENSITIVITY_2000_DPS 16.4
#define MPU_DPS_TO_RADPS (PI / 180)

#define PI 3.1415

#define MPU_CLOCK 400000

const int MPU_ADDR = 0x68;
const int SDA_PIN = 21;
const int SCL_PIN = 22;

static volatile int16_t gyro_z_raw;

/**
 * @brief Read a MPU register.
 *
 * @param[in] address mpu address 
 * @param[in] _register Register address.
 */
uint8_t mpu_read_register(uint8_t address, uint8_t _register) {
  Wire.beginTransmission(address);
  Wire.write(_register);
  Wire.endTransmission();

  Wire.requestFrom(address, (uint8_t) 1);
  return Wire.read();
}


/**
 * @brief Write a MPU register with a given value.
 *
 * @param[in] address mpu address
 * @param[in] _register Register address.
 * @param[in] value Register value.
 */
void mpu_write_register(uint8_t address, uint8_t _register, uint8_t value)
{

  Wire.beginTransmission(address);
  Wire.write(_register);
  Wire.write(value);
  Wire.endTransmission(true);  
}

/**
 * @brief Read the WHOAMI register value.
 *
 * This is a read-only register set to 0x70 after reset.
 */
uint8_t mpu_who_am_i(void)
{
  return mpu_read_register(MPU_ADDR, MPU_WHOAMI);
}

/**
 * @brief MPU-6500 board setup.
 *
 * MPU is configured as follows:
 *
 * - Reset MPU restoring default settings and wait 100 ms
 * - Reset signal path and wait 100 ms
 * - Sample Rate Divider (dix) equal to 0 where: SampleRate = InternalSample /
 *   (1 + div)
 * - Set DLPF (Dual Low Pass Filter) configuration to 0 with 250 Hz of
 *   bandwidth and InternalSample = 8 kHz
 * - Configure gyroscope's Z-axis with DLPF, -2000 dps and 16.4 LSB
 * - Configure SPI at high speed (less than 20MHz)
 * - Wait 100 ms
 */
void setup_mpu(void)
{
  Wire.begin();
  Wire.setClock(MPU_CLOCK); //comment this line with compilation issues
  
  // hardware reset
  mpu_write_register(MPU_ADDR, MPU_PWR_MGMT_1, 0x80);

  //wait 100 ms
  delay(100);

  // resets gyro, accelerometer and temporizator
  mpu_write_register(MPU_ADDR, MPU_SIGNAL_PATH_RESET, 0x07);   

  // wait 100 ms
  delay(100);
  
  mpu_write_register(MPU_ADDR, MPU_SMPLRT_DIV, 0x00);
  mpu_write_register(MPU_ADDR, MPU_CONFIG, 0x00);
  mpu_write_register(MPU_ADDR, MPU_GYRO_CONFIG, 0x18);

  // wait 100 ms
  delay(100);

}

/**
 * @brief Read gyroscope's Z-axis raw value from MPU.
 */
static int16_t mpu_read_gyro_z_raw(void)
{

  return ((mpu_read_register(MPU_ADDR, MPU_GYRO_ZOUT_H) << BYTE) |
          mpu_read_register(MPU_ADDR, MPU_GYRO_ZOUT_L));
}


/**
 * @brief Calibrate the gyroscope's Z axis.
 *
 * This function should be executed when the robot is stopped. The average of
 * gyroscope z output will be substracted from the gyro output from that
 * moment on. To write MPU registers, the SPI speed is changed to low speed.
 */
void gyro_z_calibration(void)
{
	int16_t zout_c2;
	float zout_av = 0;
	int8_t i;

	for (i = 0; i < MPU_CAL_SAMPLE_NUM; i++) {
		zout_av = ((float)mpu_read_gyro_z_raw() + zout_av) /
			  MPU_AVERAGE_FACTOR;
		delay(MPU_CAL_SAMPLE_MS);
	}
	zout_c2 = -(int16_t)(zout_av * MPU_COMPLEMENT_2_FACTOR);

	mpu_write_register(MPU_ADDR, MPU_Z_OFFS_USR_H,
			   ((uint8_t)((zout_c2 & MPU_MASK_H) >> BYTE)));
	mpu_write_register(MPU_ADDR, MPU_Z_OFFS_USR_L, (uint8_t)(zout_c2 & MPU_MASK_L));
	delay(1);
}

void update_gyro_readings(void)
{
	gyro_z_raw = mpu_read_gyro_z_raw();
}

/**
 * @brief Get gyroscope's Z-axis angular speed in bits per second.
 */
int16_t get_gyro_z_raw(void)
{
	return gyro_z_raw;
}

/**
 * @brief Get gyroscope's Z-axis angular speed in radians per second.
 */
float get_gyro_z_radps(void)
{
	return ((float)gyro_z_raw * MPU_DPS_TO_RADPS /
		MPU_GYRO_SENSITIVITY_2000_DPS);
}

/**
 * @brief Get gyroscope's Z-axis angular speed in degrees per second.
 */
float get_gyro_z_dps(void)
{
	return ((float)gyro_z_raw / MPU_GYRO_SENSITIVITY_2000_DPS);
}

/**
 * @brief Get gyroscope's Z-axis angular displacement in degrees.
 */
float get_gyro_z_degrees(void) {
  return 0;
}