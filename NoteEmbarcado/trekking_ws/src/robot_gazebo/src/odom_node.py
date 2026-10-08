#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion

from robot_interfaces.msg import UARTData 
from tf2_ros import TransformBroadcaster

class OdomPublisher(Node):

    def __init__(self):
        
        super().__init__('odom_publisher')

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.uart_data_sub = self.create_subscription(

            UARTData, 
            '/uart_data', 
            self.uart_callback, 
            10
        )
        
        self.tf_broadcaster = TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.dt = 0.0
        self.last_timestamp = 0.0

        self.timer = self.create_timer(0.05, self.odom_callback)

    def yaw_to_quaternion(self, yaw):

        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def uart_callback(self, msg):

        current_x = msg.x / 1000.0
        current_y = msg.y / 1000.0
        current_theta = msg.z / 1000.0

        dt_s = (msg.timestamp - self.last_timestamp) / 1000.0
        self.last_timestamp = msg.timestamp

        if dt_s > 0:

            global_dx = current_x - self.x
            global_dy = current_y - self.y
            global_dtheta = current_theta - self.theta

            dx_local = global_dx * math.cos(self.theta) + global_dy * math.sin(self.theta)

            self.vx = dx_local / dt_s
            self.vth = global_dtheta / dt_s

        self.x = current_x
        self.y = current_y
        self.theta = current_theta

    def odom_callback(self):
        
        vx = 0.0 
        vth = 0.0

        current_time = self.get_clock().now().to_msg()
        quat = self.yaw_to_quaternion(self.theta)

        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = quat
        
        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quat

        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth

        self.odom_pub.publish(odom)

def main(args=None):

    rclpy.init(args=args)
    node = OdomPublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()