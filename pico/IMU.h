#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include "pico/stdlib.h"

int imu_init(void);

void read_imu(float *readings);

#endif