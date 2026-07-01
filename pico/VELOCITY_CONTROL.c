#include "VELOCITY_CONTROL.h"
#include <stdbool.h>
#include "ENCODER.h"
#include "MOTOR.h"
#include "PID.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <math.h>

#define MIN_PWM 85
#define COUNT_TO_M 0.0000620149f
#define MIN_TARGET_SPEED 0.15f
#define TARGET_DEADBAND 0.02f


#define CONTROL_PERIOD_MS 20

static absolute_time_t last_time;
static absolute_time_t next_time;

static int32_t left_count = 0;
static int32_t right_count = 0;
static int32_t prev_left_count = 0;
static int32_t prev_right_count = 0;

static float measured_left = 0.0f;
static float measured_right = 0.0f;

static PID left_pid  = {160.0f, 5.0f, 15.0f, 0, 0, 0, 0, 2.0f, 0.2f};
static PID right_pid = {160.0f, 5.0f, 15.0f, 0, 0, 0, 0, 2.0f, 0.2f};



static float target_speed_left = 0.0f;
static float target_speed_right = 0.0f;

void velocity_init(void)
{


    prev_left_count = encoder_get_count(0) + encoder_get_count(2);
    prev_right_count = encoder_get_count(1) + encoder_get_count(3);

    left_pid.last_output = 0.0f;
    right_pid.last_output = 0.0f;


    last_time = get_absolute_time();
    next_time = delayed_by_ms(last_time, CONTROL_PERIOD_MS);
}




void velocity_set_target(float left_mps, float right_mps)
{
    float old_left = target_speed_left;
    float old_right = target_speed_right;

    target_speed_left = left_mps;
    target_speed_right = right_mps;

    if (fabsf(target_speed_left) < TARGET_DEADBAND) {
        target_speed_left = 0.0f;
    }
    if (fabsf(target_speed_right) < TARGET_DEADBAND) {
        target_speed_right = 0.0f;
    }

// Experimentally found that these motors can only move properly and consistently past 0.15m/s, so clamping it to that 
    if (target_speed_left > 0.0f && target_speed_left < MIN_TARGET_SPEED) {
        target_speed_left = MIN_TARGET_SPEED;
    } else if (target_speed_left < 0.0f && target_speed_left > -MIN_TARGET_SPEED) {
        target_speed_left = -MIN_TARGET_SPEED;
    }

    if (target_speed_right > 0.0f && target_speed_right < MIN_TARGET_SPEED) {
        target_speed_right = MIN_TARGET_SPEED;
    } else if (target_speed_right < 0.0f && target_speed_right > -MIN_TARGET_SPEED) {
        target_speed_right = -MIN_TARGET_SPEED;
    }

    // Minimum PWM reset in case Pi sends sudden velocity command changes
    // also lets PWM start at the minimum useful command to spin motors instead of 0
    if ((old_left >= 0.0f && target_speed_left < 0.0f) ||
        (old_left <= 0.0f && target_speed_left > 0.0f)) {
        
        left_pid.integral = 0.0f;
        left_pid.last_error = 0.0f;
        left_pid.last_measured = measured_left;

        if (target_speed_left > 0.0f) {
            left_pid.last_output = MIN_PWM;
        } else {
            left_pid.last_output = -MIN_PWM;
        }
    }

    if ((old_right >= 0.0f && target_speed_right < 0.0f) ||
        (old_right <= 0.0f && target_speed_right > 0.0f)) {

        right_pid.integral = 0.0f;
        right_pid.last_error = 0.0f;
        right_pid.last_measured = measured_right;

        if (target_speed_right > 0.0f) {
            right_pid.last_output = MIN_PWM;
        } else {
            right_pid.last_output = -MIN_PWM;
        }
    }

    if (target_speed_left == 0.0f) {
        left_pid.integral = 0.0f;
        left_pid.last_error = 0.0f;
        left_pid.last_output = 0.0f;
        left_pid.last_measured = measured_left;

    }

    if (target_speed_right == 0.0f) {
        right_pid.integral = 0.0f;
        right_pid.last_error = 0.0f;
        right_pid.last_output = 0.0f;
        right_pid.last_measured = measured_right;
    }
}




void velocity_update(void)
{

    absolute_time_t now = get_absolute_time();

    if (absolute_time_diff_us(now, next_time) > 0)
        return;
    float dt = absolute_time_diff_us(last_time, now) / 1000000.0f;

    last_time = now;
    next_time = delayed_by_ms(next_time, CONTROL_PERIOD_MS);

    left_count = encoder_get_count(0) + encoder_get_count(2);
    right_count = encoder_get_count(1) + encoder_get_count(3);

    measured_left = ((left_count - prev_left_count) * COUNT_TO_M) / dt;
    measured_right = ((right_count - prev_right_count) * COUNT_TO_M) / dt;

    prev_left_count = left_count;
    prev_right_count = right_count;

    float pid_motorleft = PID_update(&left_pid, target_speed_left, measured_left, dt);
    float pid_motorright = PID_update(&right_pid, target_speed_right, measured_right, dt);

    float cmd_left = left_pid.last_output + pid_motorleft;
    float cmd_right = right_pid.last_output + pid_motorright;

    if (cmd_left > 255.0f) cmd_left = 255.0f;
    if (cmd_left < -255.0f) cmd_left = -255.0f;

    if (cmd_right > 255.0f) cmd_right = 255.0f;
    if (cmd_right < -255.0f) cmd_right = -255.0f;

    if (target_speed_left > 0.0f) {
        if (cmd_left < MIN_PWM) cmd_left = MIN_PWM;
    } else if (target_speed_left < 0.0f) {
        if (cmd_left > -MIN_PWM) cmd_left = -MIN_PWM;
    } else {
        cmd_left = 0.0f;
    }

    if (target_speed_right > 0.0f) {
        if (cmd_right < MIN_PWM) cmd_right = MIN_PWM;
    } else if (target_speed_right < 0.0f) {
        if (cmd_right > -MIN_PWM) cmd_right = -MIN_PWM;
    } else {
        cmd_right = 0.0f;
    }

    left_pid.last_output = cmd_left;
    right_pid.last_output = cmd_right;

    bool left_forward = cmd_left >= 0.0f;
    bool right_forward = cmd_right >= 0.0f;

    float pwm_left = cmd_left;
    float pwm_right = cmd_right;

    if (pwm_left < 0.0f) pwm_left = -pwm_left;
    if (pwm_right < 0.0f) pwm_right = -pwm_right;

    motor_set_left(left_forward, (uint8_t)pwm_left);
    motor_set_right(right_forward, (uint8_t)pwm_right);
}


float velocity_get_left(void)
{
    return measured_left;
}

float velocity_get_right(void)
{
    return measured_right;
}

float velocity_difference(void)
{
    return measured_right - measured_left;
}