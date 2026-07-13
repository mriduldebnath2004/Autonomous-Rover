#include <array>
#include <cmath>
#include <sstream>
#include <string>

#include "std_msgs/msg/int64_multi_array.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <chrono>
#include <cerrno>
#include <cstring>
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
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class SerialNode : public rclcpp::Node
{
public:
    SerialNode()
    : Node("serial_node"), serial_fd_(-1)
    {

        wheel_ticks_publisher_ =
            this->create_publisher<std_msgs::msg::Int64MultiArray>(
                "/wheel_ticks", 10);

        imu_publisher_ =
            this->create_publisher<sensor_msgs::msg::Imu>(
                "/imu/data_raw", 10); 


        declare_parameter<std::string>("serial_port", "/dev/ttyS0");
        declare_parameter<int>("baud_rate", 115200);

        const std::string port =
            get_parameter("serial_port").as_string();

        const int baud_rate =
            get_parameter("baud_rate").as_int();

        open_serial_port(port, baud_rate);

        telemetry_publisher_ =
            create_publisher<std_msgs::msg::String>(
                "/pico/telemetry",
                10
            );

        cmd_vel_subscriber_ =
            create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel",
                10,
                std::bind(
                    &SerialNode::cmd_vel_callback,
                    this,
                    std::placeholders::_1
                )
            );

        // Check the serial port every 10 ms for incoming Pico data.
        serial_timer_ =
            create_wall_timer(
                10ms,
                std::bind(&SerialNode::read_serial, this)
            );

        RCLCPP_INFO(
            get_logger(),
            "Serial node started on %s at %d baud",
            port.c_str(),
            baud_rate
        );
    }

    ~SerialNode() override
    {
        // Stop the rover when this node shuts down.
        if (serial_fd_ >= 0) {
            close(serial_fd_);
        }
    }

private:

    void parse_and_publish_telemetry(const std::string &line)
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

        if (!(stream >> message_type
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
                this->get_logger(),
                "Could not parse telemetry: '%s'",
                line.c_str());

            return;
        }

        if (message_type != 'T')
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Unknown serial message: '%s'",
                line.c_str());

            return;
        }

        publish_wheel_ticks(
            encoder_0,
            encoder_1,
            encoder_2,
            encoder_3);

        publish_imu(
            ax_g,
            ay_g,
            az_g,
            gx_rad_s,
            gy_rad_s,
            gz_rad_s);
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

        wheel_ticks_publisher_->publish(message);
    }

    void publish_imu(
        double ax_g,
        double ay_g,
        double az_g,
        double gx_rad_s,
        double gy_rad_s,
        double gz_rad_s)
    {
        

        constexpr double GRAVITY = 9.80665;
        sensor_msgs::msg::Imu message;

        message.header.stamp = this->get_clock()->now();
        message.header.frame_id = "imu_link";

        // Pico sends acceleration in g.
        // sensor_msgs/Imu requires m/s^2.
        message.linear_acceleration.x = ax_g * GRAVITY;
        message.linear_acceleration.y = ay_g * GRAVITY;
        message.linear_acceleration.z = az_g * GRAVITY;

        // Pico sends angular velocity in rad/seconds already
        message.angular_velocity.x = gx_rad_s;
        message.angular_velocity.y = gy_rad_s;
        message.angular_velocity.z = gz_rad_s;

        /*
        * The MPU6050 data currently contains no orientation quaternion.
        *
        * Setting orientation_covariance[0] to -1 tells ROS that
        * orientation is unavailable.
        */
        message.orientation_covariance[0] = -1.0;

        /*
        * Zero covariance arrays mean that the covariance is unknown.
        * We can replace these with measured variances later.
        */
        message.angular_velocity_covariance.fill(0.0);
        message.linear_acceleration_covariance.fill(0.0);

        imu_publisher_->publish(message);
    }


   

    rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr
        wheel_ticks_publisher_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr
        imu_publisher_;

    void open_serial_port(
        const std::string &port,
        int baud_rate
    )

    {
        serial_fd_ = open(
            port.c_str(),
            O_RDWR | O_NOCTTY | O_NONBLOCK
        );

        if (serial_fd_ < 0) {
            throw std::runtime_error(
                "Could not open " + port + ": " +
                std::strerror(errno)
            );
        }

        termios tty{};

        if (tcgetattr(serial_fd_, &tty) != 0) {
            close(serial_fd_);
            serial_fd_ = -1;

            throw std::runtime_error(
                "Could not read serial settings: " +
                std::string(std::strerror(errno))
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
                    std::to_string(baud_rate)
                );
        }

        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);

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
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(INLCR | ICRNL | IGNCR);

        // Raw output mode.
        tty.c_oflag &= ~OPOST;

        // Nonblocking reads.
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            close(serial_fd_);
            serial_fd_ = -1;

            throw std::runtime_error(
                "Could not configure serial port: " +
                std::string(std::strerror(errno))
            );
        }

        // Remove any old bytes sitting in the serial buffers.
        tcflush(serial_fd_, TCIOFLUSH);
    }

    void cmd_vel_callback(
        const geometry_msgs::msg::Twist::SharedPtr message
    )
    {
        const double forward_velocity = message->linear.x;
        const double yaw_rate = message->angular.z;

        std::ostringstream command;

        command << std::fixed << std::setprecision(3)
                << "V "
                << forward_velocity << " "
                << yaw_rate
                << "\n";

        const std::string command_string = command.str();

        const ssize_t bytes_written = write(
            serial_fd_,
            command_string.c_str(),
            command_string.size()
        );

        if (bytes_written < 0) {
            RCLCPP_ERROR(
                get_logger(),
                "Serial write failed: %s",
                std::strerror(errno)
            );

            return;
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
            const ssize_t bytes_read = read(
                serial_fd_,
                temporary_buffer,
                sizeof(temporary_buffer)
            );

            if (bytes_read > 0) {
                receive_buffer_.append(
                    temporary_buffer,
                    static_cast<std::size_t>(bytes_read)
                );
            }
            else if (bytes_read == 0) {
                break;
            }
            else {
                // EAGAIN means no data is currently available.
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    RCLCPP_ERROR(
                        get_logger(),
                        "Serial read failed: %s",
                        std::strerror(errno)
                    );
                }

                break;
            }
        }

        // A single read may contain multiple complete telemetry lines.
        std::size_t newline_position;

        while (
            (newline_position = receive_buffer_.find('\n'))
            != std::string::npos
        ) {
            std::string line =
                receive_buffer_.substr(0, newline_position);

            receive_buffer_.erase(0, newline_position + 1);

            // Handle possible Windows-style line endings.
            if (!line.empty() && line.back() == '\r') {
                line.pop_back();
            }

            if (!line.empty()) {
                publish_telemetry(line);
            }
        }

        // Prevent unlimited growth if malformed data never contains '\n'.
        if (receive_buffer_.size() > 4096) {
            RCLCPP_WARN(
                get_logger(),
                "Serial receive buffer overflow; clearing buffer"
            );

            receive_buffer_.clear();
        }
    }

    void publish_telemetry(const std::string &line)
    {
        std_msgs::msg::String message;
        message.data = line;

        telemetry_publisher_->publish(message);

        parse_and_publish_telemetry(line);

        RCLCPP_DEBUG(
            get_logger(),
            "Received: %s",
            line.c_str()
        );
    }

    int serial_fd_;
    std::string receive_buffer_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
        telemetry_publisher_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
        cmd_vel_subscriber_;

    rclcpp::TimerBase::SharedPtr serial_timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<SerialNode>();
        rclcpp::spin(node);
    }
    catch (const std::exception &exception) {
        RCLCPP_FATAL(
            rclcpp::get_logger("serial_node"),
            "%s",
            exception.what()
        );
    }

    rclcpp::shutdown();
    return 0;
}
