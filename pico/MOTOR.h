#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"


void motor_init(void);

void motor_set_left(bool forward, uint8_t speed);

void motor_set_right(bool forward, uint8_t speed);

void motor_stop_all(void);


#endif