#!/usr/bin/env python3
"""Controle da missão: único publisher de cmd_vel no launch de cones laranja."""
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from robot_interfaces.msg import ConeDetections
from orange_core import MissionConfig, OrangeMission, Detection, Output


class OrangeMissionNode(Node):
    def __init__(self):
        super().__init__('orange_mission')
        defaults = MissionConfig()
        values = {}
        for name in defaults.__dataclass_fields__:
            self.declare_parameter(name, getattr(defaults,name), ParameterDescriptor(read_only=True))
            values[name] = self.get_parameter(name).value
        self.declare_parameter('auto_start', True, ParameterDescriptor(read_only=True))
        self.declare_parameter('publish_rate_hz', 20., ParameterDescriptor(read_only=True))
        rate = self.get_parameter('publish_rate_hz').value
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError('publish_rate_hz inválido')
        if self.get_parameter('use_sim_time').value:
            raise ValueError('Use use_sim_time=false no robô real')
        self.mission = OrangeMission(MissionConfig(**values))
        self.cmd_pub = self.create_publisher(Twist,'cmd_vel',1)
        self.led_pub = self.create_publisher(Bool,'led_command',1)
        self.center_pub = self.create_publisher(PointStamped,'cone_center',1)
        self.centered_pub = self.create_publisher(Bool,'cone_centered',1)
        self.status_pub = self.create_publisher(String,'mission/status',1)
        self.vision_sub = self.create_subscription(ConeDetections,'orange_cones',self.on_cones,qos_profile_sensor_data)
        self.lidar_sub = self.create_subscription(LaserScan,'scan',self.on_scan,qos_profile_sensor_data)
        self.start_srv = self.create_service(Trigger,'mission/start',self.start)
        self.stop_srv = self.create_service(Trigger,'mission/stop',self.stop)
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1./rate,self.tick,clock=self.steady)
        self.last_status = None
        self.last_header = None
        if self.get_parameter('auto_start').value:
            self.mission.start(time.monotonic())
        self.get_logger().info(f'Modo {values["mode"]}; chegada: {values["arrival_gap_m"]:.2f} m da frente, '
                               f'{values["camera_to_front_m"]+values["arrival_gap_m"]:.2f} m da câmera.')

    def ros_now(self):
        return self.get_clock().now().nanoseconds/1e9

    @staticmethod
    def stamp(header):
        return header.stamp.sec+header.stamp.nanosec/1e9

    def on_cones(self,msg):
        c = self.mission.c
        if (msg.image_width,msg.image_height) != (c.image_width,c.image_height):
            self.mission.frame_time = None
            self.get_logger().error('Visão e controle têm resoluções diferentes; movimento suspenso.')
            return
        detections = [Detection(d.track_id,d.center_x,d.center_y,d.width,d.height,d.confidence,d.class_id,d.depth_m,
                                d.bearing_rad,d.half_width_rad)
                      for d in msg.detections]
        if self.mission.observe(detections,self.stamp(msg.header),self.ros_now(),time.monotonic()):
            self.last_header = msg.header

    def on_scan(self,msg):
        self.mission.guard.update(msg.ranges,msg.angle_min,msg.angle_increment,msg.range_min,msg.range_max,
                                  self.stamp(msg.header),self.ros_now(),time.monotonic())

    def start(self,request,response):
        self.mission.start(time.monotonic())
        self.publish(self.mission.output())
        response.success, response.message = True,'Missão reiniciada; aguardando imagens novas.'
        return response

    def stop(self,request,response):
        self.mission.stop(time.monotonic())
        self.publish(self.mission.output())
        response.success, response.message = True,'Robô parado; use mission/start para reiniciar.'
        return response

    def publish(self,out):
        command = Twist()
        command.linear.x, command.angular.z = float(out.v),float(out.omega)
        self.cmd_pub.publish(command)
        led = Bool()
        led.data = out.led
        self.led_pub.publish(led)
        point = PointStamped()
        point.header.stamp = self.get_clock().now().to_msg()
        point.header.frame_id = 'camera_color_pixels'
        d = out.target
        fresh = (self.mission.frame_time is not None
                 and 0 <= time.monotonic()-self.mission.frame_time <= self.mission.c.detection_timeout_s)
        if d is not None and fresh and out.state not in ('IDLE','FAULT','COMPLETE'):
            if self.last_header is not None:
                point.header = self.last_header
            point.point.x, point.point.y = d.x,d.y
        else:
            point.point.x = point.point.y = -1.
        self.center_pub.publish(point)
        centered = Bool()
        centered.data = bool(d is not None and fresh and out.state not in ('IDLE','FAULT')
                             and self.mission.center_frames >= self.mission.c.confirmation_frames
                             and abs(d.x-(self.mission.c.image_width-1)/2) <= self.mission.c.tolerance_px)
        self.centered_pub.publish(centered)
        status = dict(state=out.state,reason=out.reason,visited=out.visited,total=self.mission.c.max_cones,
                      target_id=d.track_id if d else None,
                      range_source=d.range_source if d else None,
                      front_gap_m=self.mission.gap(d) if d and math.isfinite(d.depth_m) else None)
        text = String()
        text.data = json.dumps(status,allow_nan=False)
        self.status_pub.publish(text)
        key = (out.state,out.reason,out.visited)
        if key != self.last_status:
            self.get_logger().info(text.data)
            self.last_status = key

    def tick(self):
        self.publish(self.mission.step(time.monotonic(),self.ros_now()))


def main(args=None):
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = OrangeMissionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.mission.stop(time.monotonic())
                node.publish(node.mission.output())
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
