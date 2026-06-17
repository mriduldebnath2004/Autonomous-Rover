#include <stdio.h>
#include "pico/stdlib.h"
#include "PINS.h"
#include "ENCODER.h"
#include "MOTOR.h"
#include "IMU.h"
#include "PID.h"

#define MIN_PWM 85
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


    PID left_pid  = {180.0f, 5.0f, 15.0f, 0, 0, 0, MIN_PWM, 5.0};
    PID right_pid = {180.0f, 5.0f, 15.0f, 0, 0, 0, MIN_PWM, 5.0};


    float target_speed = 0.3f;

    float prev_left_count = 0;
    float prev_right_count = 0;

    float max_left_speed = 0.0f;
    float max_right_speed = 0.0f;

    float final_left_speed = 0.0f;
    float final_right_speed = 0.0f;

    float rise_time_left = -1.0f;
    float rise_time_right = -1.0f;

    float elapsed_time = 0.0f;

    while (true) {
        sleep_ms(5000);

        absolute_time_t last_time = get_absolute_time();
        absolute_time_t next_time = delayed_by_ms(last_time, 50);

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
            
            elapsed_time += dt;

            // Peak speeds
            if (measured_left > max_left_speed)
                max_left_speed = measured_left;

            if (measured_right > max_right_speed)
                max_right_speed = measured_right;

            // Rise time (first time reaching 90% target)
            if (rise_time_left < 0.0f && measured_left >= 0.9f * target_speed)
                rise_time_left = elapsed_time;

            if (rise_time_right < 0.0f && measured_right >= 0.9f * target_speed)
                rise_time_right = elapsed_time;

            // Store final value each iteration
            final_left_speed = measured_left;
            final_right_speed = measured_right;

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

            if (target_speed > 0.0f) {
                if (output_left < MIN_PWM)
                    output_left = MIN_PWM;

                if (output_right < MIN_PWM)
                    output_right = MIN_PWM;
            } else {
                if (output_left < 0.0f)
                    output_left = 0.0f;

                if (output_right < 0.0f)
                    output_right = 0.0f;
            }
            
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

        float overshoot_left =
            ((max_left_speed - target_speed) / target_speed) * 100.0f;

        float overshoot_right =
            ((max_right_speed - target_speed) / target_speed) * 100.0f;

        float sse_left = target_speed - final_left_speed;
        float sse_right = target_speed - final_right_speed;

        printf("\n===== PID RESULTS =====\n");

        printf("Target Speed: %.3f m/s\n\n", target_speed);

        printf("LEFT SIDE\n");
        printf("Rise Time: %.3f s\n", rise_time_left);
        printf("Peak Speed: %.3f m/s\n", max_left_speed);
        printf("Overshoot: %.2f %%\n", overshoot_left);
        printf("Final Speed: %.3f m/s\n", final_left_speed);
        printf("Steady-State Error: %.3f m/s\n\n", sse_left);

        printf("RIGHT SIDE\n");
        printf("Rise Time: %.3f s\n", rise_time_right);
        printf("Peak Speed: %.3f m/s\n", max_right_speed);
        printf("Overshoot: %.2f %%\n", overshoot_right);
        printf("Final Speed: %.3f m/s\n", final_right_speed);
        printf("Steady-State Error: %.3f m/s\n", sse_right);

        motor_set_left(true, 0);
        motor_set_right(true, 0);

        return 0;


    }
}

