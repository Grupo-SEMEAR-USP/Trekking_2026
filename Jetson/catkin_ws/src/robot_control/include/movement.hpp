#ifndef ROBOT_MOVEMENT_HPP
#define ROBOT_MOVEMENT_HPP

#include <ros/ros.h>
#include <cmath>
#include <unistd.h> 
#include <iostream> 
#include <string.h>
#include <chrono>
#include <tf2/utils.h>
#include <angles/angles.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#define DEG2RAD(deg) (deg * M_PI / 180.0)

class RobotMovement {

    public:

        RobotMovement(ros::NodeHandle& nh);

        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void moveStraight(double target, double speed);
        void turnAxial(double target, double speed);

    private:

        ros::NodeHandle nh;

        ros::Subscriber odom_sub;
        ros::Publisher cmd_vel_pub;

        double current_x;
        double current_y;
        double current_angle;

};

#endif