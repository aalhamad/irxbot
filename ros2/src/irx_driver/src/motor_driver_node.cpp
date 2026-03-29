#include "irx_driver/motor_driver.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>

MotorDriver::MotorDriver(const rclcpp::NodeOptions &options)
    : Node("motor_driver", options),
      serial_fd_(-1)
{
    // Declare parameters
    this->declare_parameter("serial_port", "/dev/ttyACM0");
    this->declare_parameter("baud_rate", 115200);
    this->declare_parameter("wheel_base", 0.4);
    this->declare_parameter("wheel_radius", 0.076);
    this->declare_parameter("max_rpm", 10000);
    this->declare_parameter("gear_ratio", 50.0);

    // Get parameters
    serial_port_ = this->get_parameter("serial_port").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    wheel_base_ = this->get_parameter("wheel_base").as_double();
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    max_rpm_ = this->get_parameter("max_rpm").as_int();
    gear_ratio_ = this->get_parameter("gear_ratio").as_double();

    // Open serial port
    if (!open_serial(serial_port_, baud_rate_))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", serial_port_.c_str());
        throw std::runtime_error("Serial port failed");
    }
    RCLCPP_INFO(this->get_logger(), "Serial port opened: %s", serial_port_.c_str());

    // Subscribe to /cmd_vel
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&MotorDriver::cmd_vel_callback, this, std::placeholders::_1));

    // Watchdog timer — stop motors if no cmd_vel for 1 second
    watchdog_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        [this]()
        {
            send_command(0, 0);
        });

    RCLCPP_INFO(this->get_logger(), "IRX Motor Driver started");
}

MotorDriver::~MotorDriver()
{
    send_command(0, 0);
    if (serial_fd_ >= 0)
        close(serial_fd_);
}

void MotorDriver::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // Reset watchdog
    watchdog_timer_->reset();

    // Convert cmd_vel to wheel RPMs (differential drive)
    double linear = msg->linear.x;
    double angular = msg->angular.z;

    double left_vel = linear - (angular * wheel_base_ / 2.0);
    double right_vel = linear + (angular * wheel_base_ / 2.0);

    // Convert m/s to ERPM
    // ERPM = (vel / wheel_circumference) * 60 * pole_pairs
    double wheel_circumference = 2.0 * M_PI * wheel_radius_;
    int32_t left_rpm = (int32_t)((left_vel / wheel_circumference) * 60.0 * 2.0 * gear_ratio_);
    int32_t right_rpm = (int32_t)((right_vel / wheel_circumference) * 60.0 * 2.0 * gear_ratio_);

    // Clamp to max RPM
    left_rpm = std::clamp(left_rpm, -max_rpm_, max_rpm_);
    right_rpm = std::clamp(right_rpm, -max_rpm_, max_rpm_);

    send_command(left_rpm, right_rpm);

    RCLCPP_DEBUG(this->get_logger(), "CMD: L=%d R=%d", left_rpm, right_rpm);
}

bool MotorDriver::open_serial(const std::string &port, int baud)
{
    serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd_ < 0)
        return false;

    struct termios tty;
    memset(&tty, 0, sizeof tty);

    // Use the baud parameter
    speed_t speed;
    switch (baud)
    {
    case 9600:
        speed = B9600;
        break;
    case 19200:
        speed = B19200;
        break;
    case 38400:
        speed = B38400;
        break;
    case 57600:
        speed = B57600;
        break;
    case 115200:
        speed = B115200;
        break;
    default:
        speed = B115200;
        break;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    tcsetattr(serial_fd_, TCSANOW, &tty);
    return true;
}

bool MotorDriver::send_command(int32_t left_rpm, int32_t right_rpm)
{
    if (serial_fd_ < 0)
        return false;

    char cmd[32];
    int len = snprintf(cmd, sizeof(cmd), "CMD %d %d\n", left_rpm, right_rpm);
    return write(serial_fd_, cmd, len) == len;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorDriver>());
    rclcpp::shutdown();
    return 0;
}
