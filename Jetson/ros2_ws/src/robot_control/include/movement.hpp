#ifndef ROBOT_MOVEMENT_HPP
#define ROBOT_MOVEMENT_HPP

#include <mutex>
#include <atomic>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/empty.hpp"

#define DEG2RAD(deg) ((deg) * M_PI / 180.0)
#define HW_IF_UPDATE_FREQ 50

class RobotMovement : public rclcpp::Node
{
public:
    RobotMovement();

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void waitForStartEngines();
    void moveStraight(double target, double speed);
    void turnAxial(double target, double speed);

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    std::mutex odom_mutex_;
    double current_x_, current_y_, current_yaw_;
    std::atomic<bool> engines_started_;
};

#endif  // ROBOT_MOVEMENT_HPP