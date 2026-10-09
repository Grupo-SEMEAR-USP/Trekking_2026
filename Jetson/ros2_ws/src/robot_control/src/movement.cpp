#include "movement.hpp"
#include "tf2/utils.h"
#include "angles/angles.h"

#include <chrono>
#include <thread>
#include <algorithm>

#ifndef DEG2RAD
#define DEG2RAD(x) ((x) * M_PI / 180.0)
#endif

using namespace std::chrono_literals;

RobotMovement::RobotMovement()
: Node("movement_node"),
  current_x_(0.0), current_y_(0.0), current_yaw_(0.0),
  engines_started_(false)
{
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10,
        std::bind(&RobotMovement::odomCallback, this, std::placeholders::_1));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Em ROS2 nao existe ros::topic::waitForMessage. A alternativa e assinar
    // o topico e usar uma flag ate que a mensagem chegue (ver waitForStartEngines).
    start_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "start_engines", 10,
        [this](const std_msgs::msg::Empty::SharedPtr) {
            engines_started_ = true;
        });
}

void RobotMovement::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(odom_mutex_);

    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    current_yaw_ = tf2::getYaw(q);
}

void RobotMovement::waitForStartEngines()
{
    RCLCPP_INFO(this->get_logger(), "Aguardando calibracao da IMU e ligar os motores...");
    rclcpp::Rate wait_rate(10);
    while (rclcpp::ok() && !engines_started_) {
        wait_rate.sleep();
    }
    RCLCPP_INFO(this->get_logger(), "Sinal 'start_engines' recebido! Iniciando missao.");
}

void RobotMovement::moveStraight(double target, double speed)
{
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = speed;

    rclcpp::sleep_for(100ms);

    double start_x, start_y, start_yaw;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        start_x = current_x_;
        start_y = current_y_;
        start_yaw = current_yaw_;
    }

    double traveled_distance = 0.0;
    rclcpp::Rate loop_rate(20);

    double kp = 1.0, ki = 0.0, kd = 0.1;
    double integral = 0.0, previous_yaw_err = 0.0;
    double dt = 0.05;

    while (rclcpp::ok() && traveled_distance < target) {

        double current_x_temp, current_y_temp, current_yaw_temp;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            current_x_temp = current_x_;
            current_y_temp = current_y_;
            current_yaw_temp = current_yaw_;
        }

        double yaw_err = start_yaw - current_yaw_temp;
        integral += yaw_err * dt;
        double derivative = (yaw_err - previous_yaw_err) / dt;
        double yaw_correction = (kp * yaw_err) + (ki * integral) + (kd * derivative);
        previous_yaw_err = yaw_err;

        yaw_correction = std::max(-0.5, std::min(0.5, yaw_correction));
        cmd.angular.z = yaw_correction;

        traveled_distance = std::sqrt(std::pow(current_x_temp - start_x, 2) +
                                       std::pow(current_y_temp - start_y, 2));

        cmd_vel_pub_->publish(cmd);
        RCLCPP_INFO(this->get_logger(), "Distancia percorrida: %.3f / %.3f | X_atual: %.3f",
                    traveled_distance, target, current_x_temp);
        loop_rate.sleep();
    }

    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd);
    RCLCPP_INFO(this->get_logger(), "Alvo atingido!");
}

void RobotMovement::turnAxial(double target, double speed)
{
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = (target > 0) ? std::abs(speed) : -std::abs(speed);

    double previous_yaw;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        previous_yaw = current_yaw_;
    }

    double angle_turned = 0.0;
    rclcpp::Rate loop_rate(20);

    while (rclcpp::ok() && std::abs(angle_turned) < std::abs(target)) {

        double current_yaw_temp;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            current_yaw_temp = current_yaw_;
        }

        double delta_yaw = angles::shortest_angular_distance(previous_yaw, current_yaw_temp);
        angle_turned += delta_yaw;
        previous_yaw = current_yaw_temp;

        cmd_vel_pub_->publish(cmd);
        RCLCPP_INFO(this->get_logger(), "Angulo percorrido: %.3f / %.3f",
                    std::abs(angle_turned), std::abs(target));

        loop_rate.sleep();
    }

    cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd);
    RCLCPP_INFO(this->get_logger(), "Giro finalizado!");
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto movement = std::make_shared<RobotMovement>();

    // Equivalente ao ros::AsyncSpinner(4): spin multi-thread em background,
    // para que os callbacks continuem rodando enquanto moveStraight/turnAxial bloqueiam.
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(movement);
    std::thread spin_thread([&executor]() { executor.spin(); });

    movement->waitForStartEngines();

    rclcpp::sleep_for(500ms);

    RCLCPP_INFO(movement->get_logger(), "Iniciando movimento reto...");
    movement->moveStraight(2, 0.1);

    rclcpp::sleep_for(500ms);

    RCLCPP_INFO(movement->get_logger(), "Iniciando giro...");
    movement->turnAxial(DEG2RAD(90), 0.2);

    RCLCPP_INFO(movement->get_logger(), "Sequencia finalizada.");

    rclcpp::shutdown();
    spin_thread.join();

    return 0;
}