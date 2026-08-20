#include <cstdint>
#include <cmath>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"


constexpr double COUNT_TO_METERS = 0.0000620149;
constexpr double EFFECTIVE_TRACK_WIDTH = 0.73;

/*
 * The rover's measured maximum wheel speed is approximately 0.75 m/s.
 * A 1.0 m/s limit provides some margin while still rejecting clearly
 * impossible encoder measurements.
 */
constexpr double MAX_REASONABLE_WHEEL_SPEED = 1.0;

/*
 * Expected encoder messages arrive at approximately 50 Hz, giving a
 * normal dt of approximately 0.02 seconds.
 */
constexpr double MIN_VALID_DT = 0.005;
constexpr double MAX_VALID_DT = 0.25;

/*
 * Wheel-odometry vx variance in (m/s)^2.
 * A variance of 0.01 corresponds to a standard deviation of 0.10 m/s.
 */
constexpr double VX_VARIANCE = 2.25e-4;

/*
 * The rover is nonholonomic, so its body-frame lateral velocity is
 * normally zero. Keep this covariance larger than the vx covariance
 * because a skid-steer rover can still experience lateral tire slip.
 *
 * A variance of 0.01 corresponds to a standard deviation of 0.10 m/s.
 */
constexpr double VY_VARIANCE = 1e-2;


class OdomNode : public rclcpp::Node
{
public:
    OdomNode()
    : Node("odom_node")
    {
        wheel_ticks_subscriber_ =
            this->create_subscription<
                std_msgs::msg::Int64MultiArray>(
                "/wheel_ticks",
                10,
                std::bind(
                    &OdomNode::wheel_ticks_callback,
                    this,
                    std::placeholders::_1));

        odom_publisher_ =
            this->create_publisher<
                nav_msgs::msg::Odometry>(
                "/wheel/odometry",
                10);

        RCLCPP_INFO(
            this->get_logger(),
            "Rover odometry node started");

        RCLCPP_INFO(
            this->get_logger(),
            "Maximum accepted wheel speed: %.2f m/s",
            MAX_REASONABLE_WHEEL_SPEED);
    }


private:
    bool first_callback_ = true;

    rclcpp::Time previous_time_;

    double left_last_ = 0.0;
    double right_last_ = 0.0;

    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;


    void wheel_ticks_callback(
        const std_msgs::msg::Int64MultiArray::SharedPtr message)
    {
        /*
         * A complete telemetry message must contain one cumulative
         * encoder count for each of the four motors.
         */
        if (message->data.size() != 4)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Expected 4 encoder counts, received %zu",
                message->data.size());

            return;
        }

        const rclcpp::Time current_time =
            this->get_clock()->now();

        const int64_t encoder_0 =
            message->data[0];

        const int64_t encoder_1 =
            message->data[1];

        const int64_t encoder_2 =
            message->data[2];

        const int64_t encoder_3 =
            message->data[3];

        /*
         * The Pico applies the encoder signs, so every wheel should
         * produce positive counts when the rover moves forward.
         *
         * Average the two encoders on each side. Adding them without
         * dividing by two would double the calculated distance.
         */
        const double current_left =
            COUNT_TO_METERS *
            (
                static_cast<double>(encoder_0) +
                static_cast<double>(encoder_2)
            ) /
            2.0;

        const double current_right =
            COUNT_TO_METERS *
            (
                static_cast<double>(encoder_1) +
                static_cast<double>(encoder_3)
            ) /
            2.0;

        /*
         * The first encoder message only establishes the initial
         * baseline. It does not represent new movement.
         */
        if (first_callback_)
        {
            left_last_ = current_left;
            right_last_ = current_right;
            previous_time_ = current_time;

            first_callback_ = false;

            RCLCPP_INFO(
                this->get_logger(),
                "Initial encoder baseline established");

            return;
        }

        const double dt =
            (current_time - previous_time_).seconds();

        /*
         * Reject invalid timing.
         *
         * Because the timing itself is invalid, establish a fresh
         * encoder and time baseline without publishing movement.
         */

        /*
        * Reject non-finite or non-positive time intervals.
        */
        if (
            !std::isfinite(dt) ||
            dt <= 0.0
        )
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Rejected invalid odometry timestep: %.6f s",
                dt);

            return;
        }

        /*
        * Several buffered messages can arrive almost simultaneously.
        *
        * Do not update the encoder or time baseline. The next message received
        * after enough time has elapsed will contain the accumulated encoder
        * movement over the accumulated time.
        */
        if (dt < MIN_VALID_DT)
        {
            return;
        }

        /*
        * After a long communication gap, the average velocity estimate may no
        * longer be useful. Establish a fresh baseline without publishing any
        * movement.
        */
        if (dt > MAX_VALID_DT)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Odometry gap too large: %.6f s; "
                "re-establishing baseline",
                dt);

            left_last_ = current_left;
            right_last_ = current_right;
            previous_time_ = current_time;

            return;
        }


        const double ds_left =
            current_left - left_last_;

        const double ds_right =
            current_right - right_last_;

        const double v_left =
            ds_left / dt;

        const double v_right =
            ds_right / dt;

        /*
         * Reject corrupted or physically impossible encoder data before
         * it can affect pose, velocity, or Robot Localization.
         *
         * For an encoder outlier, do not update the previous encoder
         * baseline or previous timestamp. Encoder counts are cumulative,
         * so the following correct packet can recover by being compared
         * against the last valid packet.
         */
        if (
            !std::isfinite(ds_left) ||
            !std::isfinite(ds_right) ||
            !std::isfinite(v_left) ||
            !std::isfinite(v_right) ||
            std::fabs(v_left) >
                MAX_REASONABLE_WHEEL_SPEED ||
            std::fabs(v_right) >
                MAX_REASONABLE_WHEEL_SPEED
        )
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Rejected encoder outlier: "
                "ds_left=%.4f m ds_right=%.4f m "
                "v_left=%.3f m/s v_right=%.3f m/s "
                "dt=%.6f s",
                ds_left,
                ds_right,
                v_left,
                v_right,
                dt);

            return;
        }

        /*
         * The measurement has passed all validity checks.
         */
        const double ds =
            (ds_left + ds_right) /
            2.0;

        const double dtheta =
            (ds_right - ds_left) /
            EFFECTIVE_TRACK_WIDTH;

        const double v =
            (v_left + v_right) /
            2.0;

        const double omega =
            dtheta / dt;

        /*
         * Use the rover's average heading over the timestep.
         */
        const double midpoint_heading =
            theta_ + 0.5 * dtheta;

        x_ +=
            ds * std::cos(midpoint_heading);

        y_ +=
            ds * std::sin(midpoint_heading);

        theta_ += dtheta;

        /*
         * Keep theta between -pi and +pi.
         */
        theta_ =
            std::atan2(
                std::sin(theta_),
                std::cos(theta_));

        nav_msgs::msg::Odometry odom_message;

        odom_message.header.stamp =
            current_time;

        odom_message.header.frame_id =
            "odom";

        odom_message.child_frame_id =
            "base_link";

        odom_message.pose.pose.position.x =
            x_;

        odom_message.pose.pose.position.y =
            y_;

        odom_message.pose.pose.position.z =
            0.0;

        /*
         * Convert planar yaw into a quaternion.
         */
        odom_message.pose.pose.orientation.x =
            0.0;

        odom_message.pose.pose.orientation.y =
            0.0;

        odom_message.pose.pose.orientation.z =
            std::sin(theta_ / 2.0);

        odom_message.pose.pose.orientation.w =
            std::cos(theta_ / 2.0);

        odom_message.twist.twist.linear.x =
            v;

        odom_message.twist.twist.linear.y =
            0.0;

        odom_message.twist.twist.linear.z =
            0.0;

        odom_message.twist.twist.angular.x =
            0.0;

        odom_message.twist.twist.angular.y =
            0.0;

        odom_message.twist.twist.angular.z =
            omega;

        /*
         * Robot Localization fuses body-frame vx and the nonholonomic
         * vy=0 constraint from this message.
         *
         * Twist covariance ordering is:
         * [vx, vy, vz, wx, wy, wz]. Therefore, diagonal index 0 is
         * vx variance and diagonal index 7 is vy variance.
         */
        odom_message.twist.covariance.fill(0.0);

        odom_message.twist.covariance[0] =
            VX_VARIANCE;

        odom_message.twist.covariance[7] =
            VY_VARIANCE;

        odom_publisher_->publish(
            odom_message);

        RCLCPP_DEBUG(
            this->get_logger(),
            "x=%.3f y=%.3f theta=%.3f "
            "v_left=%.3f v_right=%.3f "
            "v=%.3f omega=%.3f",
            x_,
            y_,
            theta_,
            v_left,
            v_right,
            v,
            omega);

        /*
         * Only valid measurements are allowed to become the new
         * baseline.
         */
        left_last_ =
            current_left;

        right_last_ =
            current_right;

        previous_time_ =
            current_time;
    }


    rclcpp::Subscription<
        std_msgs::msg::Int64MultiArray>::SharedPtr
        wheel_ticks_subscriber_;

    rclcpp::Publisher<
        nav_msgs::msg::Odometry>::SharedPtr
        odom_publisher_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(
        argc,
        argv);

    rclcpp::spin(
        std::make_shared<OdomNode>());

    rclcpp::shutdown();

    return 0;
}
