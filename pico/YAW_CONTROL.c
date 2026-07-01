#include "YAW_CONTROL.h"
#include "VELOCITY_CONTROL.h"
#include "PID.h"
#include "IMU.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <math.h>

#define YAW_CONTROL_PERIOD_MS 50
#define B_EFF 0.73f
#define MAX_WHEEL_SPEED 0.60f  // measured wheel max is around 0.75, accounting for external factors and battery level
#define MAX_YAW_RATE 1.2f      // actual max is around 1.6, to be safe 1.2

#define YAW_GYRO_IDX 5         // index into IMU_readings for Z-axis yaw rate (deg/s)
#define YAW_CMD_DEADBAND 0.02f

static absolute_time_t yaw_last_time;
static absolute_time_t yaw_next_time;

static float IMU_readings[6] = {0};

static float target_yaw = 0.0f;

static PID yaw_pid = {0.02f, 0.2f, 0.02f, 0, 0, 0, 0, 0.5f, 0.2f}; 

void yawrate_init(void)
{
    yaw_pid.integral     = 0.0f;
    yaw_pid.last_error   = 0.0f;
    yaw_pid.last_measured = 0.0f;
    yaw_pid.last_output  = 0.0f;

    yaw_last_time = get_absolute_time();
    yaw_next_time = delayed_by_ms(yaw_last_time, YAW_CONTROL_PERIOD_MS);
}

void yawrate_set_target(float target_rad_s)
{
    float old_yaw = target_yaw;

    target_yaw = target_rad_s;

    if (target_yaw >  MAX_YAW_RATE) target_yaw =  MAX_YAW_RATE;
    if (target_yaw < -MAX_YAW_RATE) target_yaw = -MAX_YAW_RATE;


    // Reset PID state on direction change
    if ((old_yaw > 0.0f && target_yaw < 0.0f) ||
        (old_yaw < 0.0f && target_yaw > 0.0f))
    {
        yaw_pid.integral     = 0.0f;
        yaw_pid.last_error   = 0.0f;
        yaw_pid.last_output  = 0.0f; 
    }
}

void yawrate_update(float forward_mps)
{
    absolute_time_t now = get_absolute_time();

    if (absolute_time_diff_us(now, yaw_next_time) > 0)
        return;
    float dt = absolute_time_diff_us(yaw_last_time, now) / 1000000.0f;

    yaw_last_time = now;
    yaw_next_time = delayed_by_ms(yaw_next_time, YAW_CONTROL_PERIOD_MS);

    read_imu(IMU_readings);
    float measured_yaw = IMU_readings[YAW_GYRO_IDX] * 0.0174533f;

    float correction = PID_update(&yaw_pid, target_yaw, measured_yaw, dt);
    float cmd_yaw = target_yaw + correction;

    if (cmd_yaw >  MAX_YAW_RATE) cmd_yaw =  MAX_YAW_RATE;
    if (cmd_yaw < -MAX_YAW_RATE) cmd_yaw = -MAX_YAW_RATE;
    
    if (fabsf(cmd_yaw) < YAW_CMD_DEADBAND) cmd_yaw = 0.0f;

    float turn_speed = 0.5f * B_EFF * cmd_yaw;

    float target_left  = forward_mps - turn_speed;
    float target_right = forward_mps + turn_speed;

    float max_cmd = fmaxf(fabsf(target_left), fabsf(target_right));
    if (max_cmd > MAX_WHEEL_SPEED) {
        float scale = MAX_WHEEL_SPEED / max_cmd;
        target_left  *= scale;
        target_right *= scale;
    }

    velocity_set_target(target_left, target_right);
}


