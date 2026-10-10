
#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from robot_interfaces.msg import VelocityCommand


class Twist2Ackermann(Node):

    def __init__(self):
        super().__init__('twist2ackermann')

        # --------------------------------------------------
        # PARAMETROS GEOMETRICOS
        # Valores arbitrarios: substituir pelas medidas reais.
        # Unidades: metros.
        # --------------------------------------------------
        self.declare_parameter('wheelbase', 0.30)
        self.declare_parameter('rear_track_width', 0.25)
        self.declare_parameter('wheel_radius', 0.05)

        # --------------------------------------------------
        # LIMITES PROVISORIOS DO ROBO
        # --------------------------------------------------
        self.declare_parameter('max_steering_angle', 0.60)  # rad
        self.declare_parameter('max_wheel_speed', 10.0)     # rad/s
        self.declare_parameter('command_timeout', 0.5)      # s

        self.L = float(self.get_parameter('wheelbase').value)
        self.W = float(
            self.get_parameter('rear_track_width').value
        )
        self.r = float(self.get_parameter('wheel_radius').value)

        self.max_steering_angle = float(
            self.get_parameter('max_steering_angle').value
        )
        self.max_wheel_speed = float(
            self.get_parameter('max_wheel_speed').value
        )
        self.command_timeout = float(
            self.get_parameter('command_timeout').value
        )

        # Validacao dos parametros
        if self.L <= 0.0 or self.W <= 0.0 or self.r <= 0.0:
            raise ValueError(
                'wheelbase, rear_track_width e wheel_radius '
                'devem ser positivos.'
            )

        if not (
            0.0 < self.max_steering_angle < math.pi / 2.0
        ):
            raise ValueError(
                'max_steering_angle deve estar entre 0 e pi/2 rad.'
            )

        if self.max_wheel_speed <= 0.0:
            raise ValueError('max_wheel_speed deve ser positivo.')

        if self.command_timeout <= 0.0:
            raise ValueError('command_timeout deve ser positivo.')

        # --------------------------------------------------
        # SUBSCRIBER: recebe o Twist do movement.py
        # --------------------------------------------------
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # --------------------------------------------------
        # PUBLISHER: uma mensagem com os tres valores
        # --------------------------------------------------
        self.velocity_pub = self.create_publisher(
            VelocityCommand,
            '/velocity_command',
            10
        )

        # Estado para controle de timeout
        self.last_cmd_time = self.get_clock().now()
        self.has_received_command = False
        self.last_steering_angle = 0.0

        self.timeout_timer = self.create_timer(
            0.05,
            self.check_command_timeout
        )

        self.get_logger().info(
            'twist2ackermann iniciado. '
            f'L={self.L:.3f} m, '
            f'W={self.W:.3f} m, '
            f'r={self.r:.3f} m'
        )

        self.get_logger().info(
            'Publicando os comandos em /velocity_command'
        )

    def calculate_steering_angle(self, v, omega):
        """
        Calcula o angulo de direcao equivalente pelo
        modelo bicicleta de Ackermann.

        v: velocidade longitudinal (m/s)
        omega: velocidade angular do robo (rad/s)

        Retorno: angulo de direcao em radianos.
        """
        if abs(v) < 1e-6:
            if abs(omega) < 1e-6:
                return 0.0

            # Ackermann convencional nao gira no proprio eixo.
            return math.copysign(
                self.max_steering_angle,
                omega
            )

        # Funciona para frente e para tras.
        # O caso v = 0 ja foi tratado acima.
        delta = math.atan((self.L * omega) / v)

        # Limita o angulo de direcao.
        delta = max(
            -self.max_steering_angle,
            min(self.max_steering_angle, delta)
        )

        return delta

    def calculate_wheel_speeds(self, v, omega):
        """
        Calcula as velocidades angulares das rodas traseiras.

        Retorna:
            angular_speed_left  (rad/s)
            angular_speed_right (rad/s)

        Pressupoe que v seja a velocidade no centro do eixo
        traseiro e que as rodas traseiras sejam motrizes.
        """
        # Nao permite rotacao no proprio eixo.
        if abs(v) < 1e-6:
            return 0.0, 0.0

        # Velocidades lineares das rodas (m/s).
        v_left = v - (omega * self.W / 2.0)
        v_right = v + (omega * self.W / 2.0)

        # Conversao para velocidade angular (rad/s).
        angular_speed_left = v_left / self.r
        angular_speed_right = v_right / self.r

        # Limita proporcionalmente as duas rodas,
        # preservando a relacao entre suas velocidades.
        peak = max(
            abs(angular_speed_left),
            abs(angular_speed_right)
        )

        if peak > self.max_wheel_speed:
            scale = self.max_wheel_speed / peak
            angular_speed_left *= scale
            angular_speed_right *= scale

        return angular_speed_left, angular_speed_right

    def publish_command(
        self,
        angular_speed_left,
        angular_speed_right,
        servo_angle
    ):
        """Publica os tres campos em uma unica mensagem."""
        msg = VelocityCommand()

        msg.angular_speed_left = float(angular_speed_left)
        msg.angular_speed_right = float(angular_speed_right)
        msg.servo_angle = float(servo_angle)

        self.velocity_pub.publish(msg)

    def cmd_vel_callback(self, msg):
        """Converte Twist em velocidade das rodas e direcao."""
        self.last_cmd_time = self.get_clock().now()
        self.has_received_command = True

        v = float(msg.linear.x)       # m/s
        omega = float(msg.angular.z)  # rad/s

        # Rejeita valores invalidos.
        if not math.isfinite(v) or not math.isfinite(omega):
            self.get_logger().warning(
                'cmd_vel invalido; zerando as velocidades.'
            )

            self.publish_command(
                0.0,
                0.0,
                self.last_steering_angle
            )
            return

        # Calcula o angulo de direcao.
        steering_angle = self.calculate_steering_angle(v, omega)

        # Calcula as velocidades angulares das rodas.
        angular_speed_left, angular_speed_right = (
            self.calculate_wheel_speeds(v, omega)
        )

        self.last_steering_angle = steering_angle

        # Publica os tres valores juntos.
        self.publish_command(
            angular_speed_left,
            angular_speed_right,
            steering_angle
        )

    def check_command_timeout(self):
        """
        Se /cmd_vel parar de chegar, zera as velocidades
        e mantem o ultimo angulo de direcao.
        """
        if not self.has_received_command:
            return

        elapsed = (
            self.get_clock().now() - self.last_cmd_time
        )

        if elapsed.nanoseconds > int(
            self.command_timeout * 1e9
        ):
            self.publish_command(
                0.0,
                0.0,
                self.last_steering_angle
            )

            self.has_received_command = False

            self.get_logger().warning(
                'Timeout em /cmd_vel: velocidades zeradas.'
            )


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = Twist2Ackermann()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            # Tenta enviar comando de parada ao encerrar.
            try:
                node.publish_command(
                    0.0,
                    0.0,
                    node.last_steering_angle
                )
            except Exception:
                pass

            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()