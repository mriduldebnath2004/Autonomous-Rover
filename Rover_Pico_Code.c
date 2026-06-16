#include <stdio.h>
#include "pico/stdlib.h"
#include "PINS.h"
#include "ENCODER.h"
#include "MOTOR.h"
#include "IMU.h"
#include "PID.h"

#define MIN_PWM 82
#define count_to_m 0.0000620149f

// IMU Tuning
#define AX_OFFSET  0.045f
#define AY_OFFSET -0.020f
#define AZ_OFFSET -0.050f   
#define GZ_OFFSET -0.78f

int main() {
    stdio_init_all();

    int imu_ok = imu_init();
    float imu[6];

    motor_init();
    
    encoder_init();
    
    float measured_left = 0.0f;
    float measured_right = 0.0f;

    float right_count = 0.0f;
    float left_count = 0.0f;


    PID left_pid  = {200.0f, 10.0f, 0.0f, 0, 0, 0, 85};
    PID right_pid = {200.0f, 10.0f, 0.0f, 0, 0, 0, 85};


    float target_speed = 0.5f;

    float prev_left_count = 0;
    float prev_right_count = 0;

    absolute_time_t last_time = get_absolute_time();
    absolute_time_t next_time = delayed_by_ms(last_time, 50);

    while (true) {
        sleep_ms(5000);

        prev_left_count = encoder_get_count(0) + encoder_get_count(2);
        prev_right_count = encoder_get_count(1) + encoder_get_count(3);

        for (int i = 0; i < 50; i++) {
            sleep_until(next_time);

            absolute_time_t now = get_absolute_time();
            float dt = absolute_time_diff_us(last_time, now) / 1000000.0f;

            last_time = now;
            next_time = delayed_by_ms(next_time, 50);

            left_count = encoder_get_count(0) + encoder_get_count(2);
            right_count = encoder_get_count(1) + encoder_get_count(3);

            measured_left = ((left_count - prev_left_count) * count_to_m) / dt;
            measured_right = ((right_count - prev_right_count) * count_to_m) / dt;

            prev_left_count = left_count;
            prev_right_count = right_count;

            float pid_motorleft = PID_update(&left_pid, target_speed, measured_left, dt);
            float pid_motorright = PID_update(&right_pid, target_speed, measured_right, dt);

            float output_left = left_pid.last_output + pid_motorleft;
            float output_right = right_pid.last_output + pid_motorright;

            if (output_left > 255.0f)
                output_left = 255.0f;

            if (output_right > 255.0f)
                output_right = 255.0f;

            if (output_left < 0.0f)
                output_left = 0.0f;

            if (output_right < 0.0f)
                output_right = 0.0f;
            
            left_pid.last_output = output_left;
            right_pid.last_output = output_right;
            

            motor_set_left(true, output_left);
            motor_set_right(true, output_right);

            printf(
    "Left: %.3f m/s (PWM %.1f), Right: %.3f m/s (PWM %.1f)\n",
    measured_left,
    output_left,
    measured_right,
    output_right
);
        }

        motor_set_left(true, 0);
        motor_set_right(true, 0);

        return 0;


    }
}

