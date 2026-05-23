#include "movement.hpp"

RobotMovement::RobotMovement(ros::NodeHandle& nh)
: nh(nh) 
{

    odom_sub = nh.subscribe("odom", 10, &RobotMovement::odomCallback, this);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

   
    current_x = 0.0;
    current_y = 0.0;
    current_yaw = 0.0;
}


void RobotMovement::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {

    std::lock_guard<std::mutex> lock(odom_mutex);

    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;
    
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    current_yaw = tf2::getYaw(q);
}


void RobotMovement::moveStraight(double target, double speed) {
    geometry_msgs::Twist cmd;
    cmd.linear.x = speed;
    
    ros::spinOnce();
    ros::Duration(0.1).sleep();
    ros::spinOnce();
    
    double start_x, start_y, start_yaw;

    {
        std::lock_guard<std::mutex> lock(odom_mutex);
        start_x = this->current_x;
        start_y = this->current_y;
        start_yaw = this->current_yaw;
    }

    double traveled_distance = 0.0;
    ros::Rate loop_rate(20);

    double kp = 1.0;
    double ki = 0.0;
    double kd = 0.1;
    
    double integral = 0.0;
    double previous_yaw_err = 0.0;
    
    double dt = 0.05;

    while(ros::ok() && traveled_distance < target) {

        ros::spinOnce();

        double current_x_temp, current_y_temp, current_yaw_temp;

        {
            std::lock_guard<std::mutex> lock(odom_mutex);
            current_x_temp = this->current_x;
            current_y_temp = this->current_y;
            current_yaw_temp = this->current_yaw;
        }

        double yaw_err = start_yaw - current_yaw_temp;

        integral += yaw_err * dt;
        double derivative = (yaw_err - previous_yaw_err) / dt;
        
        double yaw_correction = (kp*yaw_err) + (ki*integral) + (kd*derivative);
        previous_yaw_err = yaw_err;

        if (yaw_correction > 0.5) yaw_correction = 0.5;
        if (yaw_correction < -0.5) yaw_correction = -0.5;

        cmd.angular.z = yaw_correction;

        traveled_distance = std::sqrt(std::pow(current_x_temp - start_x, 2) + 
                                      std::pow(current_y_temp - start_y, 2));

        cmd_vel_pub.publish(cmd);
        ROS_INFO("Distancia percorrida: %.3f / %.3f | X_atual: %.3f", traveled_distance, target, current_x_temp);
        loop_rate.sleep();
    }

    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_vel_pub.publish(cmd);
    ROS_INFO("Alvo atingido!");
}


void RobotMovement::turnAxial(double target, double speed) {
    geometry_msgs::Twist cmd;
   
    cmd.linear.x = 0;   
    cmd.angular.z = (target > 0) ? std::abs(speed) : -std::abs(speed);
    
    double previous_yaw;

    {
        std::lock_guard<std::mutex> lock(odom_mutex);
        previous_yaw = current_yaw;
    }

    double angle_turned;
    ros::Rate loop_rate(20);

    while(ros::ok() && std::abs(angle_turned) < std::abs(target)) {
        
        double current_yaw_temp;

        {
            std::lock_guard<std::mutex> lock(odom_mutex);
            current_yaw_temp = current_yaw;
        }

        double delta_yaw = angles::shortest_angular_distance(previous_yaw, current_yaw_temp);
        angle_turned += delta_yaw;
        previous_yaw = current_yaw_temp;

        cmd_vel_pub.publish(cmd);
        ROS_INFO("Angulo percorrido: %.3f / %.3f", std::abs(angle_turned), std::abs(target));

        loop_rate.sleep();
    }

    cmd.angular.z = 0.0;
    cmd_vel_pub.publish(cmd);
    ROS_INFO("Giro finalizado!");
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "movement_node");
    ros::NodeHandle nh;

    RobotMovement movement(nh);

    
    ros::AsyncSpinner spinner(4);
    spinner.start();

    ROS_INFO("Aguardando calibração da IMU e ligar os motores...");
    ros::topic::waitForMessage<std_msgs::Empty>("start_engines", nh);
    ROS_INFO("Sinal 'start_engines' recebido! Iniciando missao.");

    ros::Duration(0.5).sleep();

    ROS_INFO("Iniciando movimento reto...");
    movement.moveStraight(2, 0.1);

    ros::Duration(0.5).sleep();

    ROS_INFO("Iniciando giro...");
    movement.turnAxial(DEG2RAD(90), 0.2);

    ROS_INFO("Sequencia finalizada.");
    ros::waitForShutdown();

    return 0;
}