#include <chrono>
#include <cmath>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class SerialNode : public rclcpp::Node
{
public:
    SerialNode()
    : Node("serial_node"),
      serial_fd_(-1)
    {
        declare_parameter<std::string>(
            "serial_port",
            "/dev/ttyS0"
        );

        declare_parameter<int>(
            "baud_rate",
            115200
        );

        declare_parameter<int>(
            "imu_calibration_samples",
            300
        );

        /*
         * Instead of checking gyro readings against a fixed maximum
         * value (which breaks down if the sensor has a constant
         * nonzero offset), we check whether the gyro readings are
         * VARYING over a short rolling window. A stationary rover
         * with a fixed sensor bias will have near-zero variance even
         * if its raw readings are not near zero.
         */
        declare_parameter<double>(
            "max_gyro_stddev",
            0.1
        );

        declare_parameter<int>(
            "gyro_variation_window_samples",
            20
        );

        calibration_sample_target_ =
            static_cast<std::size_t>(
                get_parameter(
                    "imu_calibration_samples"
                ).as_int()
            );

        max_gyro_stddev_ =
            get_parameter(
                "max_gyro_stddev"
            ).as_double();

        variation_window_size_ =
            static_cast<std::size_t>(
                get_parameter(
                    "gyro_variation_window_samples"
                ).as_int()
            );

        if (calibration_sample_target_ == 0) {
            throw std::runtime_error(
                "imu_calibration_samples must be greater than zero"
            );
        }

        if (variation_window_size_ == 0) {
            throw std::runtime_error(
                "gyro_variation_window_samples must be greater than zero"
            );
        }

        const std::string port =
            get_parameter(
                "serial_port"
            ).as_string();

        const int baud_rate =
            get_parameter(
                "baud_rate"
            ).as_int();

        wheel_ticks_publisher_ =
            create_publisher<
                std_msgs::msg::Int64MultiArray
            >(
                "/wheel_ticks",
                10
            );

        imu_publisher_ =
            create_publisher<
                sensor_msgs::msg::Imu
            >(
                "/imu/data_raw",
                10
            );

        telemetry_publisher_ =
            create_publisher<
                std_msgs::msg::String
            >(
                "/pico/telemetry",
                10
            );

        open_serial_port(
            port,
            baud_rate
        );

        cmd_vel_subscriber_ =
            create_subscription<
                geometry_msgs::msg::Twist
            >(
                "/cmd_vel",
                10,
                std::bind(
                    &SerialNode::cmd_vel_callback,
                    this,
                    std::placeholders::_1
                )
            );

        serial_timer_ =
            create_wall_timer(
                10ms,
                std::bind(
                    &SerialNode::read_serial,
                    this
                )
            );

        RCLCPP_INFO(
            get_logger(),
            "Serial node started on %s at %d baud",
            port.c_str(),
            baud_rate
        );

        RCLCPP_INFO(
            get_logger(),
            "Keep rover stationary while IMU calibrates "
            "using %zu samples",
            calibration_sample_target_
        );
    }

    ~SerialNode() override
    {
        if (serial_fd_ >= 0) {
            send_velocity_command(
                0.0,
                0.0
            );

            close(serial_fd_);
            serial_fd_ = -1;
        }
    }

private:
    void parse_and_publish_telemetry(
        const std::string &line)
    {
        std::istringstream stream(line);

        char message_type;

        int64_t encoder_0;
        int64_t encoder_1;
        int64_t encoder_2;
        int64_t encoder_3;

        double ax_g;
        double ay_g;
        double az_g;

        double gx_rad_s;
        double gy_rad_s;
        double gz_rad_s;

        if (!(stream
              >> message_type
              >> encoder_0
              >> encoder_1
              >> encoder_2
              >> encoder_3
              >> ax_g
              >> ay_g
              >> az_g
              >> gx_rad_s
              >> gy_rad_s
              >> gz_rad_s))
        {
            RCLCPP_WARN(
                get_logger(),
                "Could not parse telemetry: '%s'",
                line.c_str()
            );

            return;
        }

        if (message_type != 'T') {
            RCLCPP_WARN(
                get_logger(),
                "Unknown serial message: '%s'",
                line.c_str()
            );

            return;
        }

        publish_wheel_ticks(
            encoder_0,
            encoder_1,
            encoder_2,
            encoder_3
        );

        /*
        * Pico transmits gyroscope measurements in degrees per second.
        * sensor_msgs/Imu requires radians per second.
        */
        constexpr double DEG_TO_RAD =
            0.017453292519943295;

        process_and_publish_imu(
            ax_g,
            ay_g,
            az_g,
            gx_rad_s * DEG_TO_RAD,
            gy_rad_s * DEG_TO_RAD,
            gz_rad_s * DEG_TO_RAD
        );


    }

    void publish_wheel_ticks(
        int64_t encoder_0,
        int64_t encoder_1,
        int64_t encoder_2,
        int64_t encoder_3)
    {
        std_msgs::msg::Int64MultiArray message;

        message.data = {
            encoder_0,
            encoder_1,
            encoder_2,
            encoder_3
        };

        wheel_ticks_publisher_->publish(
            message
        );
    }

    void process_and_publish_imu(
        double ax_g,
        double ay_g,
        double az_g,
        double gx_rad_s,
        double gy_rad_s,
        double gz_rad_s)
    {
        /*
         * Pico telemetry units:
         *
         * Accelerometer: g
         * Gyroscope: rad/s
         */
        if (!imu_calibrated_) {
            calibrate_imu(
                gx_rad_s,
                gy_rad_s,
                gz_rad_s
            );

            /*
             * Do not publish IMU measurements until calibration
             * has completed.
             */
            return;
        }

        constexpr double GRAVITY =
            9.80665;

        sensor_msgs::msg::Imu message;

        message.header.stamp =
            get_clock()->now();

        message.header.frame_id =
            "imu_link";

        /*
         * Convert acceleration from g to m/s^2.
         */
        message.linear_acceleration.x =
            ax_g * GRAVITY;

        message.linear_acceleration.y =
            ay_g * GRAVITY;

        message.linear_acceleration.z =
            az_g * GRAVITY;

        /*
         * Remove startup gyro bias.
         *
         * Input and output remain in rad/s.
         */
        message.angular_velocity.x =
            gx_rad_s - gyro_bias_x_;

        message.angular_velocity.y =
            gy_rad_s - gyro_bias_y_;

        message.angular_velocity.z =
            gz_rad_s - gyro_bias_z_;

        /*
         * No orientation quaternion is being provided.
         */
        message.orientation_covariance.fill(
            0.0
        );

        message.orientation_covariance[0] =
            -1.0;

        /*
         * Approximate gyro variance based on:
         *
         * 0.3 deg/s = 0.00524 rad/s
         * variance = 0.00524^2 = approximately 2.7e-5
         */
        message.angular_velocity_covariance = {
            1e-2, 0.0,    0.0,
            0.0,    1e-2, 0.0,
            0.0,    0.0,  1e-2
        };

        /*
         * Acceleration covariance is currently unknown.
         */
        message.linear_acceleration_covariance.fill(
            0.0
        );

        imu_publisher_->publish(
            message
        );
    }

    void calibrate_imu(
        double gx_rad_s,
        double gy_rad_s,
        double gz_rad_s)
    {
        push_to_window(
            gx_window_,
            gx_window_sum_,
            gx_window_sum_sq_,
            gx_rad_s
        );

        push_to_window(
            gy_window_,
            gy_window_sum_,
            gy_window_sum_sq_,
            gy_rad_s
        );

        push_to_window(
            gz_window_,
            gz_window_sum_,
            gz_window_sum_sq_,
            gz_rad_s
        );

        /*
         * Wait until the rolling window is full before judging
         * whether the rover is stationary.
         */
        if (gz_window_.size() < variation_window_size_) {
            return;
        }

        const double stddev_x =
            window_stddev(
                gx_window_,
                gx_window_sum_,
                gx_window_sum_sq_
            );

        const double stddev_y =
            window_stddev(
                gy_window_,
                gy_window_sum_,
                gy_window_sum_sq_
            );

        const double stddev_z =
            window_stddev(
                gz_window_,
                gz_window_sum_,
                gz_window_sum_sq_
            );

        const double max_stddev =
            std::max({stddev_x, stddev_y, stddev_z});

        /*
         * A fixed sensor offset does not affect standard deviation,
         * only genuine variation does. This lets us detect movement
         * even when an axis has a large constant bias.
         */
        if (max_stddev > max_gyro_stddev_) {
            reset_imu_calibration();

            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Gyro variation detected (stddev=%.5f rad/s). "
                "Keep rover stationary. Calibration restarted.",
                max_stddev
            );

            return;
        }

        /*
         * Stable enough. Accumulate the raw sample toward the bias
         * average (the average itself still needs the raw, unfiltered
         * readings, not the windowed values).
         */
        gyro_sum_x_ += gx_rad_s;
        gyro_sum_y_ += gy_rad_s;
        gyro_sum_z_ += gz_rad_s;

        imu_calibration_count_++;

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Calibrating IMU: %zu/%zu samples",
            imu_calibration_count_,
            calibration_sample_target_
        );

        if (
            imu_calibration_count_ <
            calibration_sample_target_
        )
        {
            return;
        }

        const double sample_count =
            static_cast<double>(
                imu_calibration_count_
            );

        gyro_bias_x_ =
            gyro_sum_x_ / sample_count;

        gyro_bias_y_ =
            gyro_sum_y_ / sample_count;

        gyro_bias_z_ =
            gyro_sum_z_ / sample_count;

        imu_calibrated_ = true;

        constexpr double RAD_TO_DEG =
            57.29577951308232;

        RCLCPP_INFO(
            get_logger(),
            "IMU calibration complete"
        );

        RCLCPP_INFO(
            get_logger(),
            "Gyro bias [rad/s]: "
            "x=%.6f, y=%.6f, z=%.6f",
            gyro_bias_x_,
            gyro_bias_y_,
            gyro_bias_z_
        );

        RCLCPP_INFO(
            get_logger(),
            "Gyro bias [deg/s]: "
            "x=%.3f, y=%.3f, z=%.3f",
            gyro_bias_x_ * RAD_TO_DEG,
            gyro_bias_y_ * RAD_TO_DEG,
            gyro_bias_z_ * RAD_TO_DEG
        );
    }

    void push_to_window(
        std::deque<double> &window,
        double &sum,
        double &sum_sq,
        double value)
    {
        window.push_back(value);
        sum += value;
        sum_sq += value * value;

        if (window.size() > variation_window_size_) {
            const double oldest = window.front();
            window.pop_front();

            sum -= oldest;
            sum_sq -= oldest * oldest;
        }
    }

    double window_stddev(
        const std::deque<double> &window,
        double sum,
        double sum_sq) const
    {
        const double n =
            static_cast<double>(window.size());

        const double mean = sum / n;

        double variance =
            sum_sq / n - mean * mean;

        /*
         * Guard against tiny negative values caused by floating
         * point rounding.
         */
        if (variance < 0.0) {
            variance = 0.0;
        }

        return std::sqrt(variance);
    }

    void reset_imu_calibration()
    {
        imu_calibration_count_ = 0;

        gyro_sum_x_ = 0.0;
        gyro_sum_y_ = 0.0;
        gyro_sum_z_ = 0.0;

        gyro_bias_x_ = 0.0;
        gyro_bias_y_ = 0.0;
        gyro_bias_z_ = 0.0;

        imu_calibrated_ = false;

        /*
         * Clear the rolling windows too, otherwise stale samples from
         * before the movement event would bleed into the next
         * calibration attempt.
         */
        gx_window_.clear();
        gy_window_.clear();
        gz_window_.clear();

        gx_window_sum_ = 0.0;
        gy_window_sum_ = 0.0;
        gz_window_sum_ = 0.0;

        gx_window_sum_sq_ = 0.0;
        gy_window_sum_sq_ = 0.0;
        gz_window_sum_sq_ = 0.0;
    }

    void open_serial_port(
        const std::string &port,
        int baud_rate)
    {
        serial_fd_ = open(
            port.c_str(),
            O_RDWR |
            O_NOCTTY |
            O_NONBLOCK
        );

        if (serial_fd_ < 0) {
            throw std::runtime_error(
                "Could not open " +
                port +
                ": " +
                std::strerror(errno)
            );
        }

        termios tty{};

        if (
            tcgetattr(
                serial_fd_,
                &tty
            ) != 0
        )
        {
            close(serial_fd_);
            serial_fd_ = -1;

            throw std::runtime_error(
                "Could not read serial settings: " +
                std::string(
                    std::strerror(errno)
                )
            );
        }

        speed_t speed;

        switch (baud_rate) {
            case 9600:
                speed = B9600;
                break;

            case 57600:
                speed = B57600;
                break;

            case 115200:
                speed = B115200;
                break;

            default:
                close(serial_fd_);
                serial_fd_ = -1;

                throw std::runtime_error(
                    "Unsupported baud rate: " +
                    std::to_string(
                        baud_rate
                    )
                );
        }

        cfsetispeed(
            &tty,
            speed
        );

        cfsetospeed(
            &tty,
            speed
        );

        // 8 data bits.
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;

        // No parity.
        tty.c_cflag &= ~PARENB;

        // One stop bit.
        tty.c_cflag &= ~CSTOPB;

        // No hardware flow control.
        tty.c_cflag &= ~CRTSCTS;

        // Enable receiver and ignore modem control lines.
        tty.c_cflag |= CREAD | CLOCAL;

        // Raw input mode.
        tty.c_lflag &=
            ~(ICANON | ECHO | ECHOE | ISIG);

        tty.c_iflag &=
            ~(IXON | IXOFF | IXANY);

        tty.c_iflag &=
            ~(INLCR | ICRNL | IGNCR);

        // Raw output mode.
        tty.c_oflag &= ~OPOST;

        // Nonblocking reads.
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

        if (
            tcsetattr(
                serial_fd_,
                TCSANOW,
                &tty
            ) != 0
        )
        {
            close(serial_fd_);
            serial_fd_ = -1;

            throw std::runtime_error(
                "Could not configure serial port: " +
                std::string(
                    std::strerror(errno)
                )
            );
        }

        tcflush(
            serial_fd_,
            TCIOFLUSH
        );
    }

    void cmd_vel_callback(
        const geometry_msgs::msg::Twist::SharedPtr message)
    {
        /*
         * Prevent the rover from moving while the gyro bias is being
         * measured.
         */
        if (!imu_calibrated_) {
            send_velocity_command(
                0.0,
                0.0
            );

            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Ignoring cmd_vel while IMU calibrates"
            );

            return;
        }

        send_velocity_command(
            message->linear.x,
            message->angular.z
        );
    }

    void send_velocity_command(
        double forward_velocity,
        double yaw_rate)
    {
        if (serial_fd_ < 0) {
            return;
        }

        std::ostringstream command;

        command
            << std::fixed
            << std::setprecision(3)
            << "V "
            << forward_velocity
            << " "
            << yaw_rate
            << "\n";

        const std::string command_string =
            command.str();

        const ssize_t bytes_written =
            write(
                serial_fd_,
                command_string.c_str(),
                command_string.size()
            );

        if (bytes_written < 0) {
            if (
                errno == EAGAIN ||
                errno == EWOULDBLOCK
            )
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    1000,
                    "Serial output buffer is temporarily busy"
                );
            }
            else {
                RCLCPP_ERROR(
                    get_logger(),
                    "Serial write failed: %s",
                    std::strerror(errno)
                );
            }

            return;
        }

        if (
            static_cast<std::size_t>(
                bytes_written
            ) != command_string.size()
        )
        {
            RCLCPP_WARN(
                get_logger(),
                "Incomplete serial write: wrote %zd of %zu bytes",
                bytes_written,
                command_string.size()
            );
        }

        RCLCPP_DEBUG(
            get_logger(),
            "Sent: %s",
            command_string.c_str()
        );
    }

    void read_serial()
    {
        char temporary_buffer[256];

        while (true) {
            const ssize_t bytes_read =
                read(
                    serial_fd_,
                    temporary_buffer,
                    sizeof(
                        temporary_buffer
                    )
                );

            if (bytes_read > 0) {
                receive_buffer_.append(
                    temporary_buffer,
                    static_cast<std::size_t>(
                        bytes_read
                    )
                );
            }
            else if (bytes_read == 0) {
                break;
            }
            else {
                if (
                    errno != EAGAIN &&
                    errno != EWOULDBLOCK
                )
                {
                    RCLCPP_ERROR(
                        get_logger(),
                        "Serial read failed: %s",
                        std::strerror(errno)
                    );
                }

                break;
            }
        }

        std::size_t newline_position;

        while (
            (
                newline_position =
                receive_buffer_.find('\n')
            ) != std::string::npos
        )
        {
            std::string line =
                receive_buffer_.substr(
                    0,
                    newline_position
                );

            receive_buffer_.erase(
                0,
                newline_position + 1
            );

            if (
                !line.empty() &&
                line.back() == '\r'
            )
            {
                line.pop_back();
            }

            if (!line.empty()) {
                publish_telemetry(
                    line
                );
            }
        }

        if (
            receive_buffer_.size() >
            4096
        )
        {
            RCLCPP_WARN(
                get_logger(),
                "Serial receive buffer overflow; clearing buffer"
            );

            receive_buffer_.clear();
        }
    }

    void publish_telemetry(
        const std::string &line)
    {
        std_msgs::msg::String message;
        message.data = line;

        telemetry_publisher_->publish(
            message
        );

        parse_and_publish_telemetry(
            line
        );

        RCLCPP_DEBUG(
            get_logger(),
            "Received: %s",
            line.c_str()
        );
    }

    rclcpp::Publisher<
        std_msgs::msg::Int64MultiArray
    >::SharedPtr wheel_ticks_publisher_;

    rclcpp::Publisher<
        sensor_msgs::msg::Imu
    >::SharedPtr imu_publisher_;

    rclcpp::Publisher<
        std_msgs::msg::String
    >::SharedPtr telemetry_publisher_;

    rclcpp::Subscription<
        geometry_msgs::msg::Twist
    >::SharedPtr cmd_vel_subscriber_;

    rclcpp::TimerBase::SharedPtr
        serial_timer_;

    int serial_fd_;

    std::string receive_buffer_;

    bool imu_calibrated_ = false;

    std::size_t imu_calibration_count_ = 0;

    std::size_t calibration_sample_target_ = 300;

    /*
     * Maximum allowed standard deviation of gyro readings over the
     * rolling window, in rad/s, for the rover to be considered
     * stationary.
     */
    double max_gyro_stddev_ = 0.01;

    /*
     * Number of recent samples used to compute the rolling
     * standard deviation.
     */
    std::size_t variation_window_size_ = 20;

    double gyro_sum_x_ = 0.0;
    double gyro_sum_y_ = 0.0;
    double gyro_sum_z_ = 0.0;

    double gyro_bias_x_ = 0.0;
    double gyro_bias_y_ = 0.0;
    double gyro_bias_z_ = 0.0;

    // Rolling windows and running sums used for the variation check.
    std::deque<double> gx_window_;
    std::deque<double> gy_window_;
    std::deque<double> gz_window_;

    double gx_window_sum_ = 0.0;
    double gy_window_sum_ = 0.0;
    double gz_window_sum_ = 0.0;

    double gx_window_sum_sq_ = 0.0;
    double gy_window_sum_sq_ = 0.0;
    double gz_window_sum_sq_ = 0.0;
};

int main(
    int argc,
    char *argv[])
{
    rclcpp::init(
        argc,
        argv
    );

    try {
        auto node =
            std::make_shared<
                SerialNode
            >();

        rclcpp::spin(
            node
        );
    }
    catch (
        const std::exception &exception
    )
    {
        RCLCPP_FATAL(
            rclcpp::get_logger(
                "serial_node"
            ),
            "%s",
            exception.what()
        );
    }

    rclcpp::shutdown();

    return 0;
}
