#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class Twist2Ackermann(Node):

    def __init__(self):
        super().__init__('twist2ackermann')

        self.twist_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.twist_callback,
            10)
        self.twist_sub

        self.left_pub = self.create_publisher(Float64, '/Margarete/left_motor', 10)
        self.right_pub = self.create_publisher(Float64, '/Margarete/right_motor', 10)

        self.servo_pub = self.create_publisher(Float64, '/Margarete/servo_angle', 10)

        self.declare_parameter('linear_gain', 10.0)

        # distância entre eixos dianteiro / traseiro
        self.L = 1.0

        # distância entre rodas do mesmo eixo
        self.W = 1.0

    def twist_callback(self, msg):

        left = Float64()
        right = Float64()
        servo_angle = Float64()

        linear_gain = self.get_parameter('linear_gain').value

        v_x = msg.linear.x
        omega = msg.angular.z

        if abs(omega) < 0.001:

            servo_angle.data = 0.0

            left.data = v_x * linear_gain
            right.data = v_x * linear_gain

        else:

            R = v_x / omega

            delta = np.arctan(self.L / R)

            v_left = v_x - (omega * self.W / 2.0)
            v_right = v_x + (omega * self.W / 2.0)

            servo_angle.data = float(delta)
            left.data = float(v_left * linear_gain)
            right.data = float(v_right * linear_gain)

        self.left_pub.publish(left)
        self.right_pub.publish(right)
        self.servo_pub.publish(servo_angle)


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



        

