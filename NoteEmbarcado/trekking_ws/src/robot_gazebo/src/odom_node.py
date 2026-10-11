#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from robot_interfaces.msg import UARTData
from control_core import OdometryState

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        self.state = OdometryState()
        self.pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf = TransformBroadcaster(self)
        self.sub = self.create_subscription(UARTData, 'uart_data', self.on_data, 10)
    def on_data(self, msg):
        self.state.update(msg.x,msg.y,msg.z,msg.timestamp)
        s = self.state
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id,odom.child_frame_id = 'odom','base_link'
        odom.pose.pose.position.x,odom.pose.pose.position.y = s.x,s.y
        odom.pose.pose.orientation.z,odom.pose.pose.orientation.w = math.sin(s.theta/2),math.cos(s.theta/2)
        odom.twist.twist.linear.x,odom.twist.twist.angular.z = s.vx,s.vth
        # Valores provisórios: não representam calibração da incerteza na grama.
        for i,var in zip((0,7,14,21,28,35),(0.25,0.25,1e6,1e6,1e6,0.25)):
            odom.pose.covariance[i] = var
            odom.twist.covariance[i] = var
        transform = TransformStamped()
        transform.header,transform.child_frame_id = odom.header,odom.child_frame_id
        transform.transform.translation.x,transform.transform.translation.y = s.x,s.y
        transform.transform.rotation = odom.pose.pose.orientation
        self.pub.publish(odom)
        self.tf.sendTransform(transform)
        # Sem timer: nunca mascara perda de telemetria com timestamps novos.

def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
