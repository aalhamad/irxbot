#ifndef IRX_DRIVER__MOTOR_DRIVER_HPP_
#define IRX_DRIVER__MOTOR_DRIVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <string>
#include <termios.h>

class MotorDriver : public rclcpp::Node
{
public:
    explicit MotorDriver(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~MotorDriver();

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    bool open_serial(const std::string &port, int baud);
    bool send_command(int32_t left_rpm, int32_t right_rpm);

    // Parameters
    std::string serial_port_;
    int baud_rate_;
    double wheel_base_;
    double wheel_radius_;
    int max_rpm_;
    double gear_ratio_;

    // Serial
    int serial_fd_;

    // ROS2
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

#endif
