#include "movement.hpp"


RobotMovement::RobotMovement(ros::NodeHandle& nh)
: nh(nh), 
    command_timeout_(nh.createTimer(ros::Duration(0.1), 
                                    &RobotHWInterface::commandTimeoutCallback, 
                                    this, true, false))

    {

    odom_sub = nh.subscribe("odom", 10, &RobotMovement::odomCallback, this)
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    double current_x = 0.0;
    double current_y = 0.0;
    double current_angle = 0.0;

}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;
    
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    current_angle = tf2::getYaw(q);
}

void moveStraight(double target, double speed) {
    
    geometry::Twist cmd;
    cmd.linear.x = speed;
    
    double start_x = current_x;
    double start_y = current_y;
    double traveled_distance = 0.0;

    ros::Rate loop_rate(20);

    while(ros::ok() && traveled_distance < target) {
        ros::spinOnce();

        traveled_distance = std::sqrt(std::pow(current_x - start_x, 2) + 
                                      std::pow(current_y - start_y, 2));

        cmd_vel_pub.publish(cmd);
        ROS_INFO("Distancia percorrida: %f", traveled_distance);

        loop_rate.sleep();

    }

        cmd.linear.x = 0.0;
        cmd_vel_pub.publish(cmd);
        ROS_INFO("Alvo atingido!");
}


void turnAxial(double target, double speed) {

    geometry_msgs::Twist cmd;
    cmd.angular.z = (target_rad > 0) ? std::abs(speed) : -std::abs(speed);
    
    double start_angle = current_angle;
    double angle_turned = 0.0;

    ros::Rate loop_rate(20);

    while(ros::ok() && std::abs(angle_turned) < std::abs(target_rad)) {
        ros::spinOnce();

        angle_turned = angles::shortest_angular_distance(start_angle, current_angle);

        cmd_vel_pub.publish(cmd);
        ROS_INFO("Distancia percorrida: %f", angle_turned);

        loop_rate.sleep();

    }

        cmd.angular.z = 0.0;
        cmd_vel_pub.publish(cmd);
        ROS_INFO("Alvo atingido!");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "movement_node");
    ros::NodeHandle nh; // Cria um NodeHandle

    RobotMovement movement(nh); // Passa o NodeHandle como argumento

    ros::Rate rate(HW_IF_UPDATE_FREQ);
    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Duration(1.0).sleep();

    ROS_INFO("Iniciando movimento reto...");
    movement.moveStraight(0.5, 0.1);

    ros::Duration(0.5).sleep();

    ROS_INFO("Iniciando giro...");
    movement.turnAxial(DEG2RAD(90), 0.2);

    ROS_INFO("Sequencia finalizada.");

    ros::waitForShutdown();

    return 0;
}