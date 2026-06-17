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


    PID left_pid  = {180.0f, 5.0f, 15.0f, 0, 0, 0, 0, 5.0};
    PID right_pid = {180.0f, 5.0f, 15.0f, 0, 0, 0, 0, 5.0};


    float target_speed_left  = -0.3f;
    float target_speed_right = -0.3f;

    // apply minimum useful target - left motor
    if (target_speed_left > 0.0f && target_speed_left < 0.15f) {
        target_speed_left = 0.15f;
        left_pid.last_output = MIN_PWM;
    } else if (target_speed_left < 0.0f && target_speed_left > -0.15f) {
        target_speed_left = -0.15f;
        left_pid.last_output = -MIN_PWM;
    }

    // apply minimum useful target - right motor
    if (target_speed_right > 0.0f && target_speed_right < 0.15f) {
        target_speed_right = 0.15f;
        right_pid.last_output = MIN_PWM;
    } else if (target_speed_right < 0.0f && target_speed_right > -0.15f) {
        target_speed_right = -0.15f;
        right_pid.last_output = -MIN_PWM;
    }


    float prev_left_count = 0;
    float prev_right_count = 0;


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

            // update baselines for next iteration (was missing - caused runaway speed readings)
            prev_left_count = left_count;
            prev_right_count = right_count;

            float pid_motorleft = PID_update(&left_pid, target_speed_left, measured_left, dt);
            float pid_motorright = PID_update(&right_pid, target_speed_right, measured_right, dt);


                        // signed outputs
            float cmd_left = left_pid.last_output + pid_motorleft;
            float cmd_right = right_pid.last_output + pid_motorright;

            // clamp signed command
            if (cmd_left > 255.0f) cmd_left = 255.0f;
            if (cmd_left < -255.0f) cmd_left = -255.0f;

            if (cmd_right > 255.0f) cmd_right = 255.0f;
            if (cmd_right < -255.0f) cmd_right = -255.0f;

            // minimum PWM deadband, but signed - evaluated independently per side
            if (target_speed_left > 0.1f) {
                if (cmd_left < MIN_PWM) cmd_left = MIN_PWM;
            } else if (target_speed_left < -0.1f) {
                if (cmd_left > -MIN_PWM) cmd_left = -MIN_PWM;
            } else {
                cmd_left = 0.0f;
            }

            if (target_speed_right > 0.1f) {
                if (cmd_right < MIN_PWM) cmd_right = MIN_PWM;
            } else if (target_speed_right < -0.1f) {
                if (cmd_right > -MIN_PWM) cmd_right = -MIN_PWM;
            } else {
                cmd_right = 0.0f;
            }

            // store SIGNED command
            left_pid.last_output = cmd_left;
            right_pid.last_output = cmd_right;

            // convert signed command to motor direction + positive PWM
            bool left_forward = cmd_left >= 0.0f;
            bool right_forward = cmd_right >= 0.0f;

            float pwm_left = cmd_left;
            float pwm_right = cmd_right;

            if (pwm_left < 0.0f) pwm_left = -pwm_left;
            if (pwm_right < 0.0f) pwm_right = -pwm_right;

            motor_set_left(left_forward, pwm_left);
            motor_set_right(right_forward, pwm_right);

            printf(
    "Left: %.3f m/s (PWM %.1f), Right: %.3f m/s (PWM %.1f)\n",
    measured_left,
    pwm_left,
    measured_right,
    pwm_right
);
        }



        motor_set_left(true, 0);
        motor_set_right(true, 0);

        // loop repeats: idle 5s, then drive for 2.5s again
        // (removed an unconditional `return 0;` that was here - it killed
        //  main after a single burst, which contradicted the while(true)
        //  + sleep_ms(5000) cycling structure)
    }
}