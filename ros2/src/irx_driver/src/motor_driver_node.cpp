#include "irx_driver/motor_driver.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>
#include <thread>
#include <cmath>

MotorDriver::MotorDriver(const rclcpp::NodeOptions &options)
    : Node("motor_driver", options),
      serial_fd_(-1),
      x_(0.0), y_(0.0), theta_(0.0)
{
    // Declare parameters
    this->declare_parameter("serial_port", "/dev/ttyACM0");
    this->declare_parameter("baud_rate", 115200);
    this->declare_parameter("wheel_base", 0.635);
    this->declare_parameter("wheel_radius", 0.0762);
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

    // Initialize time
    last_odom_time_ = this->now();

    // TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Publishers
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    // Subscribe to /cmd_vel
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&MotorDriver::cmd_vel_callback, this, std::placeholders::_1));

    // Odometry timer — read ODO from STM32 every 50ms
    odom_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&MotorDriver::read_odometry, this));

    // Watchdog timer — stop motors if no cmd_vel
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
    watchdog_timer_->reset();

    double linear = msg->linear.x;
    double angular = msg->angular.z;

    double left_vel = linear - (angular * wheel_base_ / 2.0);
    double right_vel = linear + (angular * wheel_base_ / 2.0);

    double wheel_circumference = 2.0 * M_PI * wheel_radius_;
    int32_t left_rpm = (int32_t)((left_vel / wheel_circumference) * 60.0 * 2.0 * gear_ratio_);
    int32_t right_rpm = (int32_t)((right_vel / wheel_circumference) * 60.0 * 2.0 * gear_ratio_);

    left_rpm = std::clamp(left_rpm, -max_rpm_, max_rpm_);
    right_rpm = std::clamp(right_rpm, -max_rpm_, max_rpm_);

    send_command(left_rpm, right_rpm);
}

void MotorDriver::read_odometry()
{
    if (serial_fd_ < 0)
        return;

    char buf[64];
    int n = read(serial_fd_, buf, sizeof(buf) - 1);
    if (n <= 0)
        return;

    buf[n] = '\0';

    int32_t left_rpm = 0, right_rpm = 0;
    if (sscanf(buf, "ODO %d %d", &left_rpm, &right_rpm) != 2)
        return;

    // Convert ERPM to wheel velocity (m/s)
    double wheel_circumference = 2.0 * M_PI * wheel_radius_;
    double left_vel = (left_rpm / (60.0 * 2.0 * gear_ratio_)) * wheel_circumference;
    double right_vel = (-right_rpm / (60.0 * 2.0 * gear_ratio_)) * wheel_circumference;

    // Calculate robot velocity
    double linear = (left_vel + right_vel) / 2.0;
    double angular = (right_vel - left_vel) / wheel_base_;

    // Update pose
    rclcpp::Time now = this->now();
    double dt = (now - last_odom_time_).seconds();
    last_odom_time_ = now;

    x_ += linear * cos(theta_) * dt;
    y_ += linear * sin(theta_) * dt;
    theta_ += angular * dt;

    // Publish odometry
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = linear;
    odom.twist.twist.angular.z = angular;

    odom_pub_->publish(odom);

    // Publish TF
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.rotation = odom.pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf);
}

bool MotorDriver::open_serial(const std::string &port, int baud)
{
    serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd_ < 0)
        return false;

    struct termios tty;
    memset(&tty, 0, sizeof tty);

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

    // Wait for STM32 boot
    RCLCPP_INFO(rclcpp::get_logger("motor_driver"), "Waiting for STM32 boot...");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    RCLCPP_INFO(rclcpp::get_logger("motor_driver"), "STM32 ready!");

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