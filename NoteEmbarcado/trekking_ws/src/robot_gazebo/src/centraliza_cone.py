#!/usr/bin/env python3
"""Centraliza horizontalmente um cone com direção Ackermann, usando ROS 2.

Entrada: cone_center (geometry_msgs/PointStamped), x/y em pixels.
Saída: cmd_vel (geometry_msgs/Twist), m/s e rad/s; cone_centered (Bool).
O timestamp da entrada deve ser o da imagem, no mesmo relógio ROS.
Sem ROS instalado, Config e ConeController podem ser importados para testes.
"""

from dataclasses import dataclass
import math
import time
from typing import Optional


@dataclass(frozen=True)
class Config:
    image_width: int = 424
    image_height: int = 240
    wheelbase_m: float = 0.30  # Substituir pela distância real entre eixos.
    forward_speed_m_s: float = 0.10
    steering_gain_rad: float = 0.8
    max_steering_rad: float = 0.35  # Limite provisório, não calibração do servo.
    tolerance_px: float = 12.0
    resume_tolerance_px: float = 20.0
    confirmation_frames: int = 3
    detection_timeout_s: float = 0.4
    publish_rate_hz: float = 20.0

    def __post_init__(self):
        for name in ('image_width', 'image_height', 'confirmation_frames'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ValueError(f'{name} deve ser inteiro positivo.')
        if self.image_width < 2 or self.image_height < 2:
            raise ValueError('A imagem deve ter pelo menos 2 x 2 pixels.')
        for name in ('wheelbase_m', 'forward_speed_m_s', 'steering_gain_rad',
                     'max_steering_rad', 'tolerance_px', 'resume_tolerance_px',
                     'detection_timeout_s', 'publish_rate_hz'):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} deve ser finito e positivo.')
        if self.max_steering_rad >= math.pi / 2:
            raise ValueError('max_steering_rad deve ser menor que pi/2.')
        if not self.tolerance_px < self.resume_tolerance_px < (self.image_width - 1) / 2:
            raise ValueError('Exigir 0 < tolerance_px < resume_tolerance_px < meia largura.')


@dataclass(frozen=True)
class Command:
    linear_x: float = 0.0
    angular_z: float = 0.0
    centered: bool = False
    state: str = 'sem_deteccao'


class ConeController:
    """Controle independente de ROS; não estima distância nem escolhe cones."""

    def __init__(self, config: Config):
        self.config = config
        self.last_stamp: Optional[float] = None
        self.last_received: Optional[float] = None
        self.error_px = 0.0
        self.centered = False
        self.confirmations = 0

    def invalidate(self):
        self.last_received = None
        self.centered = False
        self.confirmations = 0

    def receive(self, x: float, y: float, stamp_s: float,
                ros_now_s: float, monotonic_now_s: float) -> bool:
        """Aceita apenas imagens novas, recentes e com centro dentro da imagem."""
        c = self.config
        if (not all(math.isfinite(v) for v in (x, y, stamp_s, ros_now_s, monotonic_now_s))
                or not 0 <= x < c.image_width or not 0 <= y < c.image_height
                or stamp_s <= 0 or not -0.05 <= ros_now_s - stamp_s <= c.detection_timeout_s):
            self.invalidate()
            return False

        # Repetir o mesmo frame não renova o timeout nem confirma centralização.
        if self.last_stamp is not None and stamp_s <= self.last_stamp:
            return False
        if self.last_received is not None and monotonic_now_s - self.last_received > c.detection_timeout_s:
            self.invalidate()

        self.last_stamp = stamp_s
        self.last_received = monotonic_now_s
        self.error_px = x - (c.image_width - 1) / 2.0
        error = abs(self.error_px)

        # Histerese: evita arrancar/parar por ruído perto da borda da tolerância.
        if self.centered and error <= c.resume_tolerance_px:
            return True
        self.centered = False
        if error <= c.tolerance_px:
            self.confirmations += 1
            self.centered = self.confirmations >= c.confirmation_frames
        else:
            self.confirmations = 0
        return True

    def command(self, monotonic_now_s: float, ros_now_s: float) -> Command:
        c = self.config
        if self.last_received is None or self.last_stamp is None:
            return Command()
        if (not math.isfinite(monotonic_now_s) or not math.isfinite(ros_now_s)
                or not 0 <= monotonic_now_s - self.last_received <= c.detection_timeout_s
                or not -0.05 <= ros_now_s - self.last_stamp <= c.detection_timeout_s):
            self.invalidate()
            return Command(state='deteccao_expirada')
        if self.centered:
            return Command(centered=True, state='centralizado')
        if abs(self.error_px) <= c.tolerance_px:
            return Command(state='confirmando_centro')

        normalized_error = self.error_px / ((c.image_width - 1) / 2.0)
        # Imagem não espelhada: x maior = cone à direita = curva ROS negativa.
        steering = max(-c.max_steering_rad,
                       min(c.max_steering_rad, -c.steering_gain_rad * normalized_error))
        # O robô precisa avançar para mudar sua orientação; não gira parado.
        speed = c.forward_speed_m_s
        yaw_rate = speed * math.tan(steering) / c.wheelbase_m
        return Command(speed, yaw_rate, False, 'centralizando')


def main(args=None):
    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from rcl_interfaces.msg import ParameterDescriptor
    from geometry_msgs.msg import PointStamped, Twist
    from std_msgs.msg import Bool

    class ConeCenteringNode(Node):
        def __init__(self):
            super().__init__('centraliza_cone')
            defaults = Config()
            values = {}
            for name in defaults.__dataclass_fields__:
                self.declare_parameter(name, getattr(defaults, name),
                                       ParameterDescriptor(read_only=True))
                values[name] = self.get_parameter(name).value
            self.controller = ConeController(Config(**values))
            # Rejeitamos uso de relógio simulado: este nó é destinado ao robô real.
            if self.get_parameter('use_sim_time').value:
                raise ValueError('Use use_sim_time:=false neste nó do robô real.')
            self.command_pub = self.create_publisher(Twist, 'cmd_vel', 1)
            self.centered_pub = self.create_publisher(Bool, 'cone_centered', 1)
            sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                    durability=DurabilityPolicy.VOLATILE)
            self.subscription = self.create_subscription(
                PointStamped, 'cone_center', self.on_center, sensor_qos)
            self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.timer = self.create_timer(1.0 / self.controller.config.publish_rate_hz,
                                          self.on_timer, clock=self.steady_clock)
            self.previous_state = None
            self.get_logger().info('Aguardando cone_center: pixel x/y e timestamp da imagem.')

        def ros_seconds(self):
            return self.get_clock().now().nanoseconds / 1e9

        def on_center(self, msg):
            stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            self.controller.receive(msg.point.x, msg.point.y, stamp_s,
                                    self.ros_seconds(), time.monotonic())

        def publish(self, command):
            msg = Twist()
            msg.linear.x = command.linear_x
            msg.angular.z = command.angular_z
            self.command_pub.publish(msg)
            status = Bool()
            status.data = command.centered
            self.centered_pub.publish(status)

        def on_timer(self):
            command = self.controller.command(time.monotonic(), self.ros_seconds())
            self.publish(command)
            if command.state != self.previous_state:
                self.get_logger().info(command.state)
                self.previous_state = command.state

    # Mantém o contexto ROS vivo durante Ctrl+C para tentar publicar a parada.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = ConeCenteringNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                if rclpy.ok():
                    node.publish(Command())
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
