#ifndef IRX_DRIVER__MOTOR_DRIVER_HPP_
#define IRX_DRIVER__MOTOR_DRIVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include <string>
#include <termios.h>

class MotorDriver : public rclcpp::Node
{
public:
    explicit MotorDriver(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~MotorDriver();

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void read_odometry();
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

    // Odometry
    double x_, y_, theta_;
    rclcpp::Time last_odom_time_;

    // ROS2
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr odom_timer_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

#endif
