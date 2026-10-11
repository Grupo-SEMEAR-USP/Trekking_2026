#!/usr/bin/env python3
"""Detecta somente cones laranja e publica centro, profundidade e ID local."""
import os
import time
import math
from dataclasses import replace
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, PolygonStamped, Point32
from sensor_msgs.msg import Image
from robot_interfaces.msg import ConeDetection, ConeDetections
from ultralytics import YOLO
from orange_core import orange_class_ids, box_detection, cone_depth_m, OrangeTracker


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision')
        defaults = dict(cam_type='realsense', image_width=640, image_height=480, fps=30,
                        camera_index=0, confidence=.65, target_gate_px=90., target_lost_s=2.,
                        show_image=False, publish_debug_image=True,
                        depth_min_m=.10, depth_max_m=10., depth_min_samples=8,
                        model_path=os.path.join(get_package_share_directory('robot_gazebo'), 'best_openvino_model'))
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        self.p = {name: self.get_parameter(name).value for name in defaults}
        if min(self.p['image_width'], self.p['image_height'], self.p['fps'], self.p['depth_min_samples']) <= 0:
            raise ValueError('Dimensões/fps/amostras inválidos')
        if (not 0 < self.p['confidence'] <= 1 or self.p['target_gate_px'] <= 0 or self.p['target_lost_s'] <= 0
                or not 0 < self.p['depth_min_m'] < self.p['depth_max_m']):
            raise ValueError('Parâmetros de visão inválidos')
        if self.p['cam_type'] not in ('realsense', 'cv2'):
            raise ValueError('cam_type: realsense ou cv2')
        self.model = YOLO(self.p['model_path'], task='detect')
        self.orange_ids = orange_class_ids(self.model.names)
        self.get_logger().info(f'Classes laranja: {self.orange_ids}; centro = ponto médio da caixa.')
        self.tracker = OrangeTracker(self.p['target_gate_px'], self.p['target_lost_s'])
        self.pub = self.create_publisher(ConeDetections, 'orange_cones', 1)
        self.preview_pub = self.create_publisher(PointStamped, 'orange_cone_center', 1)
        self.metric_pub = self.create_publisher(PolygonStamped, 'cones_camera', 1)
        self.image_pub = self.create_publisher(Image, 'orange_cones/debug_image', 1)
        self.pipeline = self.cap = None
        self.last_color_number = self.last_depth_number = None
        if self.p['cam_type'] == 'realsense':
            import pyrealsense2 as rs
            self.rs = rs
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.color, self.p['image_width'], self.p['image_height'], rs.format.bgr8, self.p['fps'])
            config.enable_stream(rs.stream.depth, self.p['image_width'], self.p['image_height'], rs.format.z16, self.p['fps'])
            try:
                self.pipeline.start(config)
            except RuntimeError as error:
                self.pipeline = None
                raise RuntimeError('A RealSense não abriu o perfil RGB+depth configurado. '
                                   'Confira conexão USB e perfis com rs-enumerate-devices. ' + str(error)) from error
            self.align = rs.align(rs.stream.color)
        else:
            self.cap = cv2.VideoCapture(self.p['camera_index'])
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.p['image_width'])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.p['image_height'])
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if not self.cap.isOpened():
                raise RuntimeError('Câmera não abriu')
        self.timer = self.create_timer(1./self.p['fps'], self.tick)
        self.last_error_log = 0.

    def publish(self, detections, stamp, image=None, intrinsics=None):
        message = ConeDetections()
        message.header.stamp, message.header.frame_id = stamp, 'camera_color_pixels'
        message.image_width, message.image_height = self.p['image_width'], self.p['image_height']
        metric = PolygonStamped()
        metric.header.stamp, metric.header.frame_id = stamp, 'camera_color_optical_frame'
        for d in detections:
            msg = ConeDetection()
            msg.track_id, msg.class_id = d.track_id, d.class_id
            msg.center_x, msg.center_y, msg.width, msg.height = d.x, d.y, d.width, d.height
            msg.confidence, msg.depth_m = d.confidence, d.depth_m
            msg.bearing_rad, msg.half_width_rad = d.bearing_rad,d.half_width_rad
            message.detections.append(msg)
            if intrinsics is not None and math.isfinite(d.depth_m):
                xyz = self.rs.rs2_deproject_pixel_to_point(intrinsics, [d.x, d.y], d.depth_m)
                point = Point32()
                point.x, point.y, point.z = map(float, xyz)
                metric.polygon.points.append(point)
        self.pub.publish(message)
        self.metric_pub.publish(metric)
        chosen = min(detections, key=lambda d: abs(d.x-(self.p['image_width']-1)/2), default=None)
        center = PointStamped()
        center.header = message.header
        center.point.x, center.point.y = (chosen.x, chosen.y) if chosen else (-1., -1.)
        center.point.z = 0.  # Pixels, jamais distância; depth_m está na mensagem acima.
        self.preview_pub.publish(center)
        if image is not None and (self.p['show_image'] or self.p['publish_debug_image']):
            debug = image.copy()
            cx = int((self.p['image_width']-1)/2)
            cv2.line(debug, (cx, 0), (cx, debug.shape[0]-1), (255, 255, 255), 1)
            for d in detections:
                p1 = int(d.x-d.width/2), int(d.y-d.height/2)
                p2 = int(d.x+d.width/2), int(d.y+d.height/2)
                cv2.rectangle(debug, p1, p2, (0, 140, 255), 2)
                cv2.circle(debug, (round(d.x), round(d.y)), 5, (0, 255, 0), -1)
                label = f'laranja #{d.track_id} ({d.x:.0f},{d.y:.0f})'
                label += f' z={d.depth_m:.2f}m' if math.isfinite(d.depth_m) else ' SEM PROFUNDIDADE'
                cv2.putText(debug, label, (max(0,p1[0]), max(18,p1[1]-5)),
                            cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 255, 0), 1)
            if self.p['publish_debug_image']:
                debug = np.ascontiguousarray(debug)
                msg = Image()
                msg.header = message.header
                msg.height, msg.width = debug.shape[:2]
                msg.encoding, msg.is_bigendian, msg.step = 'bgr8', 0, msg.width*3
                msg.data = debug.tobytes()
                self.image_pub.publish(msg)
            if self.p['show_image']:
                cv2.imshow('Cones laranja: verde = centro', debug)
                cv2.waitKey(1)

    def tick(self):
        try:
            depth = intrinsics = color_intrinsics = None
            if self.pipeline is not None:
                frames = self.pipeline.poll_for_frames()
                if not frames:
                    return  # Sem frame novo, não renova timestamp.
                stamp = self.get_clock().now().to_msg()
                frames = self.align.process(frames)
                color, depth_frame = frames.get_color_frame(), frames.get_depth_frame()
                if not color or not depth_frame:
                    self.publish([], stamp)
                    return
                number = color.get_frame_number()
                if number == self.last_color_number:
                    return
                self.last_color_number = number
                image = np.asanyarray(color.get_data())
                color_intrinsics = color.profile.as_video_stream_profile().get_intrinsics()
                depth_number = depth_frame.get_frame_number()
                if depth_number != self.last_depth_number:
                    depth = np.asanyarray(depth_frame.get_data())
                    depth_scale = depth_frame.get_units()
                    intrinsics = depth_frame.profile.as_video_stream_profile().get_intrinsics()
                    self.last_depth_number = depth_number
            else:
                ok, image = self.cap.read()
                stamp = self.get_clock().now().to_msg()
                if not ok:
                    self.publish([], stamp)
                    return
            if image.shape[:2] != (self.p['image_height'], self.p['image_width']):
                self.publish([], stamp)
                raise ValueError('Resolução diverge da configuração de visão/controle')
            result = self.model.predict(image, classes=self.orange_ids, conf=self.p['confidence'], verbose=False)[0]
            detections = []
            for box in result.boxes:
                class_id = int(box.cls[0].item())
                if class_id not in self.orange_ids:
                    continue
                d = box_detection(box.xyxy[0].tolist(), float(box.conf[0].item()), class_id,
                                  self.p['image_width'], self.p['image_height'])
                if d is None:
                    continue
                if color_intrinsics is not None:
                    fx,ppx = color_intrinsics.fx,color_intrinsics.ppx
                    left = -math.atan((d.x-d.width/2-ppx)/fx)
                    right = -math.atan((d.x+d.width/2-ppx)/fx)
                    d = replace(d,bearing_rad=-math.atan((d.x-ppx)/fx),half_width_rad=(left-right)/2)
                if depth is not None:
                    d = replace(d, depth_m=cone_depth_m(depth, d, depth_scale, self.p['depth_min_m'],
                                                       self.p['depth_max_m'], self.p['depth_min_samples']))
                detections.append(d)
            detections = self.tracker.update(detections, time.monotonic())
            self.publish(detections, stamp, image, intrinsics)
        except (RuntimeError, ValueError, cv2.error) as error:
            self.publish([], self.get_clock().now().to_msg())
            now = time.monotonic()
            if now-self.last_error_log > 2:
                self.get_logger().error(str(error))
                self.last_error_log = now

    def destroy_node(self):
        if self.pipeline is not None:
            self.pipeline.stop()
        if self.cap is not None:
            self.cap.release()
        if self.p['show_image']:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VisionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

