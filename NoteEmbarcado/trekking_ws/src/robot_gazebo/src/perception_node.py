#!/usr/bin/env python3

import numpy as np
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PolygonStamped, Point32, PoseStamped
from sensor_msgs.msg import LaserScan
from message_filters import Subscriber, ApproximateTimeSynchronizer

class PerceptionNode(Node):

    def __init__(self):
        super().__init__('perception')

        self.sub_lidar = Subscriber(
            self,
            LaserScan,
            '/scan', 
            qos_profile= qos_profile_sensor_data
        )

        self.sub_vision = Subscriber(
            self,
            PolygonStamped,
            'Margarete/sensors/vision',
        )

        self.sync = ApproximateTimeSynchronizer(
            [self.sub_vision, self.sub_lidar],
            queue_size= 10,
            slop= 0.1
        )

        self.sync.registerCallback(self.sync_callback)

        self.nav_cone_goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)


    def sync_callback(self, vision_msg, lidar_msg):

        TOLERANCE_RADIUS = 0.3

        confirmed_points = []

        cone_fused = Point32()

        for vision_cone in vision_msg.polygon.points:

            vision_x = vision_cone.x
            vision_y = vision_cone.y

            vision_dist = math.hypot(vision_x, vision_y)
            vision_angle = math.atan2(vision_y, vision_x)

            if vision_dist == 0.0:
                continue

            if vision_dist > TOLERANCE_RADIUS:
                angle_openn = math.asin(TOLERANCE_RADIUS / vision_dist)

            else:
                angle_openn = math.pi / 4

            roi_max_angle = vision_angle + angle_openn
            roi_min_angle = vision_angle - angle_openn

            max_lidar_idx = int((roi_max_angle - lidar_msg.angle_min) / lidar_msg.angle_increment) 
            min_lidar_idx = int((roi_min_angle - lidar_msg.angle_min) / lidar_msg.angle_increment)   

            for i in range(min_lidar_idx, max_lidar_idx + 1):

                lidar_dists = lidar_msg.ranges[i]

                if math.isinf(lidar_dists) or math.isnan(lidar_dists) or lidar_dists < lidar_msg.range_min or lidar_dists > lidar_msg.range_max:
                    continue

                angle = lidar_msg.angle_min + (i * lidar_msg.angle_increment)

                lidar_x = lidar_dists * math.cos(angle)
                lidar_y = lidar_dists * math.sin(angle)

                dist_err = math.hypot(vision_x - lidar_x, vision_y - lidar_y)

                if dist_err <= TOLERANCE_RADIUS:

                    confirmed_points.append((lidar_x, lidar_y))

            if len(confirmed_points) > 0:

                target_cone = min(confirmed_points, key=lambda p: math.hypot(p[0], p[1]))
            
                target_x = target_cone[0]
                target_y = target_cone[1]

                total_dist = math.hypot(target_x, target_y)

                STOP_DIST = 0.8

                if total_dist > STOP_DIST:

                    factor = (total_dist - STOP_DIST) / total_dist
                    
                    target_x = target_x * factor
                    target_y = target_y * factor

                else:

                    target_x = 0.0
                    target_y = 0.0

                cone_goal_msg = PoseStamped()
                cone_goal_msg.header.stamp = self.get_clock().now().to_msg()
                cone_goal_msg.header.frame_id = lidar_msg.header.frame_id 

                cone_goal_msg.pose.position.x = target_x
                cone_goal_msg.pose.position.y = target_y
                cone_goal_msg.pose.position.z = 0.0

                cone_goal_msg.pose.orientation.x = 0.0
                cone_goal_msg.pose.orientation.y = 0.0
                cone_goal_msg.pose.orientation.z = 0.0
                cone_goal_msg.pose.orientation.w = 1.0
                
                self.nav_cone_goal_pub.publish(cone_goal_msg)


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

        