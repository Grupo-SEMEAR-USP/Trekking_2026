#!/usr/bin/env python3
"""Confirma cones métricos no LiDAR; não publica metas de navegação automaticamente."""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PolygonStamped,Point32,PointStamped
from sensor_msgs.msg import LaserScan
from message_filters import Subscriber,ApproximateTimeSynchronizer
from tf2_ros import Buffer,TransformListener,TransformException
import tf2_geometry_msgs  # Registra transformação PointStamped.

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception')
        self.declare_parameter('association_radius',0.3)
        self.radius=float(self.get_parameter('association_radius').value)
        if not math.isfinite(self.radius) or self.radius<=0:
            raise ValueError('association_radius inválido')
        self.buffer=Buffer()
        self.listener=TransformListener(self.buffer,self)
        self.lidar=Subscriber(self,LaserScan,'scan',qos_profile=qos_profile_sensor_data)
        self.vision=Subscriber(self,PolygonStamped,'cones_camera',qos_profile=qos_profile_sensor_data)
        self.sync=ApproximateTimeSynchronizer([self.vision,self.lidar],10,0.15)
        self.sync.registerCallback(self.on_pair)
        self.pub=self.create_publisher(PolygonStamped,'cones_confirmed',1)
    def on_pair(self,vision,scan):
        output=PolygonStamped()
        output.header=scan.header
        if not scan.ranges or scan.angle_increment<=0:
            self.pub.publish(output)
            return
        for cone in vision.polygon.points:
            point=PointStamped()
            point.header=vision.header
            point.point.x,point.point.y,point.point.z=cone.x,cone.y,cone.z
            try:
                # Usa TF e o timestamp da imagem; nunca compara eixos ópticos com XY do LiDAR.
                transformed=self.buffer.transform(point,scan.header.frame_id).point
            except TransformException:
                continue
            if not all(math.isfinite(v) for v in (transformed.x,transformed.y)):
                continue
            matches=[]
            for index,distance in enumerate(scan.ranges):
                if not math.isfinite(distance) or not scan.range_min<=distance<=scan.range_max:
                    continue
                angle=scan.angle_min+index*scan.angle_increment
                x,y=distance*math.cos(angle),distance*math.sin(angle)
                error=math.hypot(x-transformed.x,y-transformed.y)
                if error<=self.radius:
                    matches.append((error,x,y))
            if matches:
                _,x,y=min(matches)
                p=Point32()
                p.x,p.y=x,y
                if not any(math.hypot(p.x-q.x,p.y-q.y)<0.1 for q in output.polygon.points):
                    output.polygon.points.append(p)
        self.pub.publish(output)

def main(args=None):
    rclpy.init(args=args)
    node=PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__=='__main__':
    main()
