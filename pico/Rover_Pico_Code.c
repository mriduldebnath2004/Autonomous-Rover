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
    bool was_connected = false;

    while (true) {
        communication_receive();
        bool connected = communication_is_connected();

        if (connected) {
            forward = communication_get_forward_mps();
            yaw = communication_get_yaw_rate_radps();

            yawrate_set_target(yaw);
            yawrate_update(forward);
            velocity_update();
        } else {
            if (was_connected) {
                // connection just dropped so zero cleanly through the existing
                // reset logic in these setters rather than fighting the PIDs
                yawrate_set_target(0.0f);
                velocity_set_target(0.0f, 0.0f);
            }
            motor_stop_all();
        }
        was_connected = connected;

        communication_transmit();
    }
}
