#include <stdio.h>
#include "pico/stdlib.h"
#include "PINS.h"
#include "ENCODER.h"
#include "MOTOR.h"
#include "IMU.h"

#define MIN_PWM 82
#define count_to_m 0.0000620149f

#define AX_OFFSET  0.045f
#define AY_OFFSET -0.020f
#define AZ_OFFSET -0.050f   // because AZ is 0.95, should be 1.00
#define GZ_OFFSET -0.78f

int main() {
    stdio_init_all();

    int imu_ok = imu_init();

    float imu[6];

    while (true) {


    
        if (imu_ok) {
            read_imu(imu);
            imu[0] -= AX_OFFSET;
            imu[1] -= AY_OFFSET;
            imu[2] -= AZ_OFFSET;
            imu[5] -= GZ_OFFSET;
            printf("AX: %.2f AY: %.2f AZ: %.2f GZ: %.2f\n",
       imu[0], imu[1], imu[2], imu[5]);
        }
        else {printf("IMU initiation failed");}

        sleep_ms(500);
    }
}
