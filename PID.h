#ifndef PID_H
#define PID_H
#include "pico/stdlib.h"

typedef struct {
    float kp;
    float ki;
    float kd;

    float integral;
    float last_error;
    float last_measured;

    float last_output;
    float integral_limit;
} PID;

float PID_update(PID *pid, float target, float measured, float dt);

#endif