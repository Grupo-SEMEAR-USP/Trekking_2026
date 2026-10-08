#!/usr/bin/env python3

import numpy as np
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PolygonStamped
from sensor_msgs.msg import LaserScan
from message_filters import Subscriber, ApproximateTimeSynchronizer

class PerceptionNode(Node):

    def __init__(self):
        super().__init__('perception')

        self.lidar_subscription = Subscriber(
            LaserScan,
            '/scan', 
            self.scan_callback,
            10
        )

        self.vision_subscription = Subscriber(
            self,
            PolygonStamped,
            'Margarete/sensors/vision',
            self.vision_callback,
            10
        )



    def scan_callback(self, msg):

        valid_points = []

        # arco total de 120°
        rad_limit = math.radians(60)

        for i, dist in enumerate(msg.ranges):
            
            if math.isinf(dist) or math.isnan(dist) or dist < msg.range_min or dist > msg.range_max:
                continue
            
            angle_radius = msg.angle_min + (i * msg.angle_increment)
            angle_norm = math.atan2(math.sin(angle_radius), math.cos(angle_radius))

            if abs(angle_norm) <= rad_limit:

                x = dist * math.cos(angle_norm) 
                y = dist * math.sin(angle_norm)

                valid_points.append((x, y))

        