#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

from robot_interfaces.msg import VelocityData

class Twist2Ackermann(Node):

    def __init__(self):
        super().__init__('twist2ackermann')

        self.declare_parameter('wheel_base', 0.148245)     # [m] Entre-eixos real da Margarete
        self.declare_parameter('track_width', 0.1680)       # [m] Bitola traseira
        self.declare_parameter('wheel_radius', 0.033)       # [m] Raio da roda (33 mm)
        self.declare_parameter('servo_center', 90.0)        # [deg] Centro neutro do servo
        self.declare_parameter('max_steer_deg', 30.0)       # [deg] Limite max de esterço

        self.L = float(self.get_parameter('wheel_base').value)
        self.W = float(self.get_parameter('track_width').value)
        self.r = float(self.get_parameter('wheel_radius').value)
        self.servo_center = float(self.get_parameter('servo_center').value)
        self.max_steer_deg = float(self.get_parameter('max_steer_deg').value)

        # Assina /cmd_vel (ou /Margarete/cmd_vel)
        self.twist_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.twist_callback,
            10
        )

        # Publica no tópico de baixo nível da ESP32
        self.vel_cmd_pub = self.create_publisher(
            VelocityData,
            '/velocity_command',
            10
        )

        # Tópicos legados / float individuais
        self.left_pub = self.create_publisher(Float64, '/Margarete/left_motor', 10)
        self.right_pub = self.create_publisher(Float64, '/Margarete/right_motor', 10)
        self.servo_pub = self.create_publisher(Float64, '/Margarete/servo_angle', 10)

        self.get_logger().info(
            f"twist2ackermann iniciado: L={self.L*1000:.1f}mm, W={self.W*1000:.1f}mm, R={self.r*1000:.1f}mm | Ouvindo /cmd_vel -> /velocity_command"
        )

    def twist_callback(self, msg):
        v_x = msg.linear.x
        omega = msg.angular.z

        # 1. Ângulo de esterço Ackermann (delta)
        if abs(v_x) < 0.001:
            # Robô parado ou girando no lugar (Ackermann não gira parado, mas esterça as rodas)
            if abs(omega) > 0.001:
                delta_rad = np.arctan(self.L * omega / 0.1)
            else:
                delta_rad = 0.0
            v_left = 0.0
            v_right = 0.0
        else:
            R = v_x / omega if abs(omega) > 1e-4 else np.inf
            delta_rad = np.arctan(self.L / R)

            # Diferencial de velocidade nas rodas traseiras
            v_left = v_x - (omega * self.W / 2.0)
            v_right = v_x + (omega * self.W / 2.0)

        # Converte esterço para graus e centraliza em 90°
        steer_deg = np.degrees(delta_rad)
        steer_deg = np.clip(steer_deg, -self.max_steer_deg, self.max_steer_deg)
        servo_angle = self.servo_center + steer_deg

        # Velocidade angular de cada roda em rad/s (w = v / r)
        w_left = v_left / self.r
        w_right = v_right / self.r

        # Publica para a ESP32 (VelocityData)
        vel_cmd = VelocityData()
        vel_cmd.angular_speed_left = float(w_left)
        vel_cmd.angular_speed_right = float(w_right)
        vel_cmd.servo_angle = float(servo_angle)
        self.vel_cmd_pub.publish(vel_cmd)

        # Publica Float64 legados
        l_msg, r_msg, s_msg = Float64(), Float64(), Float64()
        l_msg.data = float(w_left)
        r_msg.data = float(w_right)
        s_msg.data = float(servo_angle)
        self.left_pub.publish(l_msg)
        self.right_pub.publish(r_msg)
        self.servo_pub.publish(s_msg)


def main(args=None):

    rclpy.init(args=args)
    node = Twist2Ackermann()
    
    try:
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()



        

