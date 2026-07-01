#include "pico/stdlib.h"
#include "IMU.h"
#include "MOTOR.h"
#include "ENCODER.h"
#include "VELOCITY_CONTROL.h"
#include "YAW_CONTROL.h"
#include "COMMUNICATION.h"


int main(void)
{


    imu_init();
    motor_init();
    encoder_init();
    velocity_init();
    yawrate_init();
    communication_init();

    float forward = 0.0f;
    float yaw = 0.0f;

    while (true) {
        communication_receive();
        forward = communication_get_forward_mps();
        yaw = communication_get_yaw_rate_radps();

        yawrate_set_target(yaw);
        yawrate_update(forward);
        velocity_update();

        communication_transmit();


        }
}
