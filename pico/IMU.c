#include "IMU.h"
#include "PINS.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdint.h>

// IMU Tuning
#define AX_OFFSET  0.045f
#define AY_OFFSET -0.020f
#define AZ_OFFSET -0.050f   

#define GX_OFFSET 0.0f
#define GY_OFFSET 0.0f
#define GZ_OFFSET -0.78f



static void imu_write_reg(uint8_t reg, uint8_t value) //write to imu fcn
{
    uint8_t buf[2] = {reg, value};
    i2c_write_blocking(i2c1, 0x68, buf, 2, false);
}


static void read_reg(uint8_t reg, uint8_t *data, uint8_t len) // read register fcn
{ 
    i2c_write_blocking(i2c1, 0x68, &reg, 1, true);
    i2c_read_blocking(i2c1, 0x68, data, len, false);
}



int imu_init(void) {
    i2c_init(i2c1, 400 * 1000); //400khz, pico i2c can handle

    gpio_set_function(IMU_SDA, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL, GPIO_FUNC_I2C);

    gpio_pull_up(IMU_SDA);
    gpio_pull_up(IMU_SCL);

    uint8_t who = 0;
    read_reg(0x75, &who,1); //read who am i register

    if (who != 0x68) {return 0;} //mpu6050 imu should return 0x68

    imu_write_reg(0x6B, 0x01); //Turn on IMU sensors, use more consistent accel X clock

    imu_write_reg(0x1A, 0x03); //config IMU LPF to 44 hz

    imu_write_reg(0x1B, 0x00); //config IMU Gyro to +-250 deg
    imu_write_reg(0x1C, 0x00); //config IMU Accel to +-2g

    return 1; 
    
}   

void read_imu(float *readings) {
    uint8_t imu_readings[12] = {0};

    read_reg(0x3B, imu_readings, 6);
    read_reg(0x43, imu_readings + 6, 6);

    for (int i = 0; i < 6; i++) {
        int16_t raw = (int16_t)((imu_readings[i * 2] << 8) |
                                 imu_readings[i * 2 + 1]);

        if (i < 3) {
            readings[i] = raw / 16384.0f; // accel in g
        } else {
            readings[i] = (raw / 131.0f);   // gyro in deg/s
        }
    }

    readings[0] -= AX_OFFSET;
    readings[1] -= AY_OFFSET;
    readings[2] -= AZ_OFFSET;

    readings[3] -= GX_OFFSET;
    readings[4] -= GY_OFFSET;
    readings[5] -= GZ_OFFSET;
}

