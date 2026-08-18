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
 * Wheel-odometry vx variance in (m/s)^2.
 * A variance of 0.01 corresponds to a standard deviation of 0.10 m/s.
 * Tune this using repeated measured-vx data.
 */
constexpr double VX_VARIANCE = 0.01;

class OdomNode : public rclcpp::Node
{
public:
    OdomNode()
    : Node("odom_node")
    {
        wheel_ticks_subscriber_ =
            this->create_subscription<std_msgs::msg::Int64MultiArray>(
                "/wheel_ticks",
                10,
                std::bind(
                    &OdomNode::wheel_ticks_callback,
                    this,
                    std::placeholders::_1));

        odom_publisher_ =
            this->create_publisher<nav_msgs::msg::Odometry>(
                "/wheel/odometry",
                10);

        RCLCPP_INFO(
            this->get_logger(),
            "Rover odometry node started");
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

        const int64_t encoder_0 = message->data[0];
        const int64_t encoder_1 = message->data[1];
        const int64_t encoder_2 = message->data[2];
        const int64_t encoder_3 = message->data[3];

        /*
         * This assumes the Pico already applies the correct encoder signs,
         * so all wheels produce positive counts while the rover moves forward.
         */
        const double current_left =
            COUNT_TO_METERS *
            (static_cast<double>(encoder_0) +
             static_cast<double>(encoder_2)) /
            2.0;

        const double current_right =
            COUNT_TO_METERS *
            (static_cast<double>(encoder_1) +
             static_cast<double>(encoder_3)) /
            2.0;

        /*
         * The first encoder message only establishes the initial baseline.
         * Otherwise, starting from non-zero encoder counts would create a
         * large false movement.
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

        if (dt <= 0.0)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Invalid odometry timestep: %f",
                dt);

            return;
        }

        const double ds_left =
            current_left - left_last_;

        const double ds_right =
            current_right - right_last_;

        const double ds =
            (ds_left + ds_right) / 2.0;

        const double dtheta =
            (ds_right - ds_left) /
            EFFECTIVE_TRACK_WIDTH;

        const double v_left =
            ds_left / dt;

        const double v_right =
            ds_right / dt;

        const double v =
            (v_left + v_right) / 2.0;

        const double omega =
            dtheta / dt;

        /*
         * Use the rover's average heading during the timestep.
         */
        const double midpoint_heading =
            theta_ + 0.5 * dtheta;

        x_ += ds * std::cos(midpoint_heading);
        y_ += ds * std::sin(midpoint_heading);
        theta_ += dtheta;

        /*
         * Keep theta between -pi and pi.
         */
        theta_ =
            std::atan2(
                std::sin(theta_),
                std::cos(theta_));

        nav_msgs::msg::Odometry odom_message;

        odom_message.header.stamp = current_time;
        odom_message.header.frame_id = "odom";
        odom_message.child_frame_id = "base_link";

        odom_message.pose.pose.position.x = x_;
        odom_message.pose.pose.position.y = y_;
        odom_message.pose.pose.position.z = 0.0;

        /*
         * Convert planar yaw angle into a quaternion.
         */
        odom_message.pose.pose.orientation.x = 0.0;
        odom_message.pose.pose.orientation.y = 0.0;
        odom_message.pose.pose.orientation.z =
            std::sin(theta_ / 2.0);
        odom_message.pose.pose.orientation.w =
            std::cos(theta_ / 2.0);

        odom_message.twist.twist.linear.x = v;
        odom_message.twist.twist.linear.y = 0.0;
        odom_message.twist.twist.linear.z = 0.0;

        odom_message.twist.twist.angular.x = 0.0;
        odom_message.twist.twist.angular.y = 0.0;
        odom_message.twist.twist.angular.z = omega;

        /*
         * robot_localization only fuses vx from this message.
         * Index 0 represents vx in the 6x6 twist covariance matrix.
         */
        odom_message.twist.covariance.fill(0.0);
        odom_message.twist.covariance[0] = VX_VARIANCE;

        odom_publisher_->publish(odom_message);

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

        left_last_ = current_left;
        right_last_ = current_right;
        previous_time_ = current_time;
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
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<OdomNode>());

    rclcpp::shutdown();

    return 0;
}
