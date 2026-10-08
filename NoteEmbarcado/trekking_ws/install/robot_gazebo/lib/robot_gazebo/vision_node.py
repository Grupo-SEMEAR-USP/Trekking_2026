#!/usr/bin/env python3

import os
import cv2
from ultralytics import YOLO
import pyrealsense2 as realsense
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped, Point32
from ament_index_python.packages import get_package_share_directory

class VisionNode(Node):

    def __init__(self):
        super().__init__('vision')

        pkg_path  = get_package_share_directory('robot_gazebo')
        model_path = os.path.join(pkg_path, 'best.pt')

        self.declare_parameter('cam_type', 'cv2')
        self.cam_type = self.get_parameter('cam_type').get_parameter_value().string_value

        self.model = YOLO(model_path)

        # ~31 FPS (numero primo para o veteras :) )
        att_time = 0.0341

        if self.cam_type == 'realsense':

            self.pipeline = realsense.pipeline()
            config = realsense.config()
            config.enable_stream(realsense.stream.depth, 640, 480, realsense.format.z16, 30)
            config.enable_stream(realsense.stream.color, 640, 480, realsense.format.bgr8, 30)

            profile = self.pipeline.start(config)

            align_to = realsense.stream.color
            self.align = realsense.align(align_to)

            self.depth_intrinsics = profile.get_stream(realsense.stream.depth).as_video_stream_profile().get_intrinsics()

            self.timer = self.create_timer(att_time, self.rs_timer_callback)

        else:

            self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
            if not self.cap.isOpened():
                self.get_logger().error("falha ao acessar webcam")

            self.timer = self.create_timer(att_time, self.cv2_timer_callback)

        self.cone_vector_publisher = self.create_publisher(PolygonStamped, 'Margarete/sensors/vision', 10)

    def rs_timer_callback(self):

        cone_vector = PolygonStamped()

        frame = self.pipeline.wait_for_frames()

        aligned_frames = self.align.process(frame)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            return

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        results = self.model(color_image, conf=0.75)

        for box in results[0].boxes:

            cone = Point32()

            xc, yc, w, h = box.xywh[0].tolist()

            x_pixel = float(xc)
            y_pixel = float(yc)

            roi = depth_image[y_pixel-2 : y_pixel+3, x_pixel-2 : x_pixel+3]

            valid = roi[roi > 0]

            dist = 0.0
            if len(valid) > 0:
                dist_median_mm = np.median(valid)
                dist = dist_median_mm * depth_frame.get_units()

            if dist <= 0.0:
                continue

            point_3d = realsense.rs2_deproject_pixel_to_point(self.depth_intrinsics, [x_pixel, y_pixel], dist)

            cone.x = point_3d[0]
            cone.y = point_3d[1]
            cone.z = point_3d[2]

            cone_vector.polygon.points.append(cone)
            cone_vector.header.stamp = self.get_clock().now().to_msg()
            cone_vector.header.frame_id = "camera_link"


        self.cone_vector_publisher.publish(cone_vector)

    def cv2_timer_callback(self):
        
        cone_vector = PolygonStamped()

        cone_vector.header.stamp = self.get_clock().now().to_msg()
        cone_vector.header.frame_id = "camera_link"
        
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('falha ao capturar imagem')
            return

        results = self.model(frame, verbose=False)
        
        for box in results[0].boxes:
        
            cone = Point32()

            xc, yc, w, h = box.xywh[0].tolist()

            cone.x = float(xc)
            cone.y = float(yc)
            
            cone_vector.polygon.points.append(cone)
            cone_vector.header.stamp = self.get_clock().now().to_msg()
            cone_vector.header.frame_id = "camera_link"

        self.cone_vector_publisher.publish(cone_vector)

        cv2.imshow("Visao Margarete", results[0].plot())
        cv2.waitKey(1)

    def destroy_node(self):
        
        if self.cam_type == 'realsense':
            self.pipeline.stop()

        else:
            if hasattr(self, 'cap'):
                self.cap.release()
                
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):

    rclpy.init()

    node = VisionNode()

    rclpy.spin(node)


if __name__ == '__main__':
    main()