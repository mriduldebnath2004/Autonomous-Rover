#include "pico/stdlib.h"
#include "PID.h"



float PID_update(PID *pid, float target, float measured, float dt) {


    float error = target - measured;
    float alpha = 0.2;

    float measured_filtered = alpha * measured + (1.0f - alpha) * (*pid).last_measured;

    (*pid).integral += error * dt;
    if ((*pid).integral > (*pid).integral_limit)
        (*pid).integral = (*pid).integral_limit;

    if ((*pid).integral < -(*pid).integral_limit)
        (*pid).integral = -(*pid).integral_limit;

    float derivative = - (measured_filtered - (*pid).last_measured) / dt;


    float output = (*pid).kp * error + 
                   (*pid).ki * (*pid).integral + 
                   (*pid).kd * derivative;



    (*pid).last_error = error;
    (*pid).last_measured = measured_filtered;

    return output;

}


