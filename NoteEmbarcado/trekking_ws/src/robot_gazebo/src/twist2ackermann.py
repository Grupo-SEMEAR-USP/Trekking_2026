#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist
from robot_interfaces.msg import VelocityCommand
from control_core import ackermann, CommandLease

class Twist2Ackermann(Node):
    def __init__(self):
        super().__init__('twist2ackermann')
        defaults = dict(wheelbase=0.30, rear_track_width=0.195, wheel_radius=0.0475,
                        max_steering_angle=0.35, max_wheel_speed=10., command_timeout=0.3)
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        self.p = {name: float(self.get_parameter(name).value) for name in defaults}
        ackermann(0., 0., *self.geometry())
        if not math.isfinite(self.p['command_timeout']) or self.p['command_timeout'] <= 0:
            raise ValueError('command_timeout inválido')
        self.lease = CommandLease(self.p['command_timeout'])
        self.pub = self.create_publisher(VelocityCommand, 'velocity_command', 1)
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.on_command, 1)
        self.clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.02, self.tick, clock=self.clock)
    def geometry(self):
        return tuple(self.p[k] for k in ('wheelbase','rear_track_width','wheel_radius',
                                        'max_steering_angle','max_wheel_speed'))
    def on_command(self, msg):
        try:
            self.lease.update(ackermann(msg.linear.x, msg.angular.z, *self.geometry()), time.monotonic())
        except ValueError:
            self.lease.clear()
    def tick(self):
        msg = VelocityCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.angular_speed_left, msg.angular_speed_right, msg.servo_angle = self.lease.get(time.monotonic())
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Twist2Ackermann()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.lease.clear()
        if rclpy.ok():
            node.tick()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
