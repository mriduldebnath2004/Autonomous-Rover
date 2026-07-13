#include "YAW_CONTROL.h"
#include "VELOCITY_CONTROL.h"
#include "PID.h"
#include "IMU.h"
#include "pico/stdlib.h"

#include <math.h>
#include <stdio.h>

#define YAW_CONTROL_PERIOD_MS 50

#define B_EFF 0.73f
#define MAX_WHEEL_SPEED 0.60f
#define MAX_YAW_RATE 1.2f

#define YAW_GYRO_IDX 5

#define YAW_CMD_DEADBAND 0.02f
#define FORWARD_STOP_DEADBAND 0.01f

static absolute_time_t yaw_last_time;
static absolute_time_t yaw_next_time;

static float IMU_readings[6] = {0};

static float target_yaw = 0.0f;

static PID yaw_pid = {
    0.02f,   // kp
    0.2f,    // ki
    0.02f,   // kd
    0.0f,    // integral
    0.0f,    // last_error
    0.0f,    // last_measured
    0.0f,    // last_output
    0.5f,    // integral_limit
    0.2f     // derivative low-pass alpha
};

void yawrate_init(void)
{
    yaw_pid.integral = 0.0f;
    yaw_pid.last_error = 0.0f;
    yaw_pid.last_measured = 0.0f;
    yaw_pid.last_output = 0.0f;

    target_yaw = 0.0f;

    yaw_last_time = get_absolute_time();
    yaw_next_time =
        delayed_by_ms(yaw_last_time, YAW_CONTROL_PERIOD_MS);
}

void yawrate_set_target(float target_rad_s)
{
    float old_yaw = target_yaw;

    target_yaw = target_rad_s;

    /*
     * Treat very small requested yaw rates as zero.
     */
    if (fabsf(target_yaw) < YAW_CMD_DEADBAND)
    {
        target_yaw = 0.0f;
    }

    /*
     * Clamp requested yaw rate to the safe operating limit.
     */
    if (target_yaw > MAX_YAW_RATE)
    {
        target_yaw = MAX_YAW_RATE;
    }
    else if (target_yaw < -MAX_YAW_RATE)
    {
        target_yaw = -MAX_YAW_RATE;
    }

    /*
     * Reset the yaw PID when the requested rotation direction changes.
     */
    if ((old_yaw > 0.0f && target_yaw < 0.0f) ||
        (old_yaw < 0.0f && target_yaw > 0.0f))
    {
        yaw_pid.integral = 0.0f;
        yaw_pid.last_error = 0.0f;
        yaw_pid.last_output = 0.0f;
    }
}

void yawrate_update(float forward_mps)
{
    absolute_time_t now = get_absolute_time();

    /*
     * Only run the yaw controller at its scheduled update rate.
     */
    if (absolute_time_diff_us(now, yaw_next_time) > 0)
    {
        return;
    }

    float dt =
        absolute_time_diff_us(yaw_last_time, now) /
        1000000.0f;

    yaw_last_time = now;

    /*
     * Schedule from the current time so the controller does not rapidly
     * execute several times if the main loop temporarily falls behind.
     */
    yaw_next_time =
        delayed_by_ms(now, YAW_CONTROL_PERIOD_MS);

    /*
     * Protect against an invalid or extremely small time step.
     */
    if (dt <= 0.0f)
    {
        return;
    }

    read_imu(IMU_readings);

    /*
     * The IMU provides yaw rate in degrees per second.
     * Convert it to radians per second.
     */
    float measured_yaw =
        IMU_readings[YAW_GYRO_IDX] *
        0.01745329252f;

    /*
     * Explicit stationary bypass:
     *
     * If both the requested forward speed and requested yaw rate are zero,
     * do not let gyro bias or noise generate wheel motion.
     */
    if (fabsf(forward_mps) < FORWARD_STOP_DEADBAND &&
        fabsf(target_yaw) < YAW_CMD_DEADBAND)
    {
        yaw_pid.integral = 0.0f;
        yaw_pid.last_error = 0.0f;
        yaw_pid.last_output = 0.0f;
        yaw_pid.last_measured = measured_yaw;

        velocity_set_target(0.0f, 0.0f);
        return;
    }

    /*
     * Keep the yaw controller active when moving forward, even when the
     * requested yaw rate is zero. This lets the rover correct its heading
     * and drive straight.
     */
    float correction =
        PID_update(
            &yaw_pid,
            target_yaw,
            measured_yaw,
            dt);

    float cmd_yaw =
        target_yaw + correction;

    /*
     * Clamp the final corrected yaw-rate command.
     */
    if (cmd_yaw > MAX_YAW_RATE)
    {
        cmd_yaw = MAX_YAW_RATE;
    }
    else if (cmd_yaw < -MAX_YAW_RATE)
    {
        cmd_yaw = -MAX_YAW_RATE;
    }

    /*
     * Remove tiny residual yaw corrections.
     */
    if (fabsf(cmd_yaw) < YAW_CMD_DEADBAND)
    {
        cmd_yaw = 0.0f;
    }

    /*
     * Differential-drive conversion:
     *
     * left  = forward - B/2 * yaw_rate
     * right = forward + B/2 * yaw_rate
     */
    float turn_speed =
        0.5f * B_EFF * cmd_yaw;

    float target_left =
        forward_mps - turn_speed;

    float target_right =
        forward_mps + turn_speed;

    /*
     * Scale both wheel commands proportionally if either command exceeds
     * the configured maximum wheel speed.
     */
    float max_cmd =
        fmaxf(
            fabsf(target_left),
            fabsf(target_right));

    if (max_cmd > MAX_WHEEL_SPEED)
    {
        float scale =
            MAX_WHEEL_SPEED / max_cmd;

        target_left *= scale;
        target_right *= scale;
    }

    velocity_set_target(
        target_left,
        target_right);
}
