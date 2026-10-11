#!/usr/bin/env python3
"""
waypoint_navigator.py
=====================

Navegação sequencial pelas 4 bases (losango) da pista de Trekking.

Estimativa de estado
--------------------
* Posição (x, y): Odometria (``/odom`` ou ``/Margarete/odom`` / ground truth).
* Orientação (yaw): IMU da RealSense D435i (``/camera/imu/data``).
    - Se a mensagem traz orientação válida (Gazebo), usa o quaternion.
    - Se ``orientation_covariance[0] == -1`` (D435i real, que só tem acel+giro),
      integra ``angular_velocity.z``.
    - Na primeira amostra, o yaw da IMU é alinhado ao yaw da odometria
      (offset), assim os dois ficam no mesmo referencial dos waypoints.

Lei de controle (robô Ackermann)
--------------------------------
* Vetor até a base:  d = (xb - x, yb - y)
* Distância:         rho = ||d||
* Rumo desejado:     theta_d = atan2(dy, dx)
* Erro angular:      dtheta = wrap(theta_d - yaw)
* v = v_max * sat(rho / r_slow) * max(cos(dtheta), 0), limitado a [v_min, v_max]
* w = k_theta * dtheta, limitado a |w| <= v / R_min (raio mínimo de giro)

Como um carro Ackermann não gira no lugar, se a base cair dentro do círculo
de giro mínimo o robô dá ré (manobra) até ela voltar a ser alcançável.

Critério de aceitação
---------------------
rho <= goal_tolerance (0.35 m)  ->  base marcada como alcançada, o robô para
por ``dwell_time`` segundos e passa automaticamente para a próxima base.

Tópicos publicados
------------------
* ``cmd_vel_topic`` (geometry_msgs/Twist): comando para o robô.
* ``~/target_heading``   (std_msgs/Float64): rumo desejado [rad] -> referência p/ PID.
* ``~/heading_error``    (std_msgs/Float64): dtheta [rad].
* ``~/distance_to_goal`` (std_msgs/Float64): rho [m].
* ``~/active_waypoint``  (std_msgs/Int32):   índice da base ativa (-1 = fim).
* ``~/goal_point``       (geometry_msgs/PointStamped): base ativa (RViz).
* ``~/status``           (std_msgs/String):  eventos (base alcançada, fim...).
"""

import math
from enum import Enum

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, Int32, String


# --------------------------------------------------------------------------- #
# Funções auxiliares
# --------------------------------------------------------------------------- #
def normalize_angle(angle: float) -> float:
    """Normaliza um ângulo para o intervalo (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q) -> float:
    """Extrai o yaw (rotação em Z) de um quaternion geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class NavState(Enum):
    WAITING_DATA = 0   # ainda sem odom/IMU (ou dados atrasados)
    DRIVING = 1        # perseguindo a base ativa
    REVERSING = 2      # manobra de ré (base dentro do círculo de giro)
    DWELLING = 3       # parado sobre a base recém-alcançada
    FINISHED = 4       # todas as bases visitadas


# --------------------------------------------------------------------------- #
# Nó principal
# --------------------------------------------------------------------------- #
class WaypointNavigator(Node):

    def __init__(self):
        super().__init__('waypoint_navigator')

        # ---------------- Parâmetros ----------------
        # Bases do losango (mesma ordem/coords do trekking.sdf):
        self.declare_parameter('waypoints_x', [5.0, 0.0, -5.0])
        self.declare_parameter('waypoints_y', [0.0, -5.0, 0.0])
        self.declare_parameter('waypoint_names', ['base_east', 'base_south', 'base_west'])
        self.declare_parameter('goal_tolerance', 0.60)
        self.declare_parameter('dwell_time', 1.0)
        self.declare_parameter('loop', False)

        self.declare_parameter('odom_topic', '/Margarete/odom')
        self.declare_parameter('imu_topic', '/camera/imu/data')
        self.declare_parameter('cmd_vel_topic', '/Margarete/cmd_vel')
        self.declare_parameter('frame_id', 'odom')

        self.declare_parameter('origin_x', 0.0)
        self.declare_parameter('origin_y', 5.0)
        self.declare_parameter('origin_yaw', 0.0)

        self.declare_parameter('use_imu_heading', True)
        self.declare_parameter('align_imu_to_odom', True)

        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('data_timeout', 1.5)
        self.declare_parameter('max_linear_speed', 0.8)
        self.declare_parameter('min_linear_speed', 0.2)
        self.declare_parameter('slowdown_radius', 2.0)
        self.declare_parameter('reverse_speed', 0.4)
        self.declare_parameter('max_reverse_time', 4.0)
        self.declare_parameter('k_heading', 1.5)

        self.declare_parameter('wheel_base', 1.0)
        self.declare_parameter('max_steering_angle', 0.6)

        gp = self.get_parameter
        xs = list(gp('waypoints_x').value)
        ys = list(gp('waypoints_y').value)
        names = list(gp('waypoint_names').value)
        if len(xs) != len(ys) or len(xs) == 0:
            raise ValueError('waypoints_x e waypoints_y devem ter o mesmo tamanho (> 0)')
        if len(names) != len(xs):
            names = [f'base_{i + 1}' for i in range(len(xs))]
        self.waypoints = list(zip(xs, ys))
        self.waypoint_names = names

        self.goal_tolerance = float(gp('goal_tolerance').value)
        self.dwell_time = float(gp('dwell_time').value)
        self.loop = bool(gp('loop').value)
        self.frame_id = gp('frame_id').value
        self.origin_x = float(gp('origin_x').value)
        self.origin_y = float(gp('origin_y').value)
        self.origin_yaw = float(gp('origin_yaw').value)
        self.use_imu_heading = bool(gp('use_imu_heading').value)
        self.align_imu_to_odom = bool(gp('align_imu_to_odom').value)
        self.data_timeout = float(gp('data_timeout').value)
        self.v_max = float(gp('max_linear_speed').value)
        self.v_min = float(gp('min_linear_speed').value)
        self.r_slow = float(gp('slowdown_radius').value)
        self.v_rev = float(gp('reverse_speed').value)
        self.max_reverse_time = float(gp('max_reverse_time').value)
        self.k_heading = float(gp('k_heading').value)

        # Raio mínimo de giro. Mesma fórmula usada pelo gz AckermannSteering:
        # R_min = L / sin(delta_max)
        wheel_base = float(gp('wheel_base').value)
        max_steer = float(gp('max_steering_angle').value)
        self.r_min = wheel_base / math.sin(max_steer)

        # ---------------- Estado ----------------
        self.x = self.y = 0.0
        self.odom_yaw = 0.0
        self.odom_stamp = None          # rclpy.time.Time da última odom

        self.imu_yaw_raw = 0.0          # yaw "cru" da IMU (quaternion ou integrado)
        self.imu_yaw_offset = None      # alinhamento IMU -> referencial dos waypoints
        self.imu_stamp = None
        self.last_gyro_time = None      # p/ integração quando não há orientação
        self.imu_has_orientation = None

        self.current_idx = 0
        self.state = NavState.WAITING_DATA
        self.dwell_until = None
        self.reverse_since = None

        # ---------------- Comunicação ----------------
        self.cmd_pub = self.create_publisher(Twist, gp('cmd_vel_topic').value, 10)
        self.target_heading_pub = self.create_publisher(Float64, '~/target_heading', 10)
        self.heading_error_pub = self.create_publisher(Float64, '~/heading_error', 10)
        self.distance_pub = self.create_publisher(Float64, '~/distance_to_goal', 10)
        self.active_wp_pub = self.create_publisher(Int32, '~/active_waypoint', 10)
        self.goal_pub = self.create_publisher(PointStamped, '~/goal_point', 10)
        self.status_pub = self.create_publisher(String, '~/status', 10)

        self.create_subscription(Odometry, gp('odom_topic').value,
                                 self.odom_callback, 10)
        self.create_subscription(Imu, gp('imu_topic').value,
                                 self.imu_callback, qos_profile_sensor_data)

        self.timer = self.create_timer(1.0 / float(gp('control_rate').value),
                                       self.control_loop)

        wp_str = ', '.join(f'{n}({x:.2f}, {y:.2f})'
                           for n, (x, y) in zip(self.waypoint_names, self.waypoints))
        self.get_logger().info(
            f'Waypoint navigator iniciado | {len(self.waypoints)} bases: {wp_str} | '
            f'tolerância={self.goal_tolerance:.2f} m | R_min={self.r_min:.2f} m')

    # ===================================================================== #
    # Callbacks de sensores
    # ===================================================================== #
    def odom_callback(self, msg: Odometry):
        """Converte a pose da odometria para o referencial dos waypoints."""
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        self.x = self.origin_x + c * ox - s * oy
        self.y = self.origin_y + s * ox + c * oy
        self.odom_yaw = normalize_angle(
            yaw_from_quaternion(msg.pose.pose.orientation) + self.origin_yaw)
        self.odom_stamp = self.get_clock().now()

    def imu_callback(self, msg: Imu):
        """Atualiza o yaw a partir da IMU da RealSense."""
        # Convenção sensor_msgs/Imu: covariance[0] == -1 -> sem orientação
        has_orientation = msg.orientation_covariance[0] >= 0.0
        if self.imu_has_orientation is None:
            self.imu_has_orientation = has_orientation
            self.get_logger().info(
                'IMU: usando ' + ('orientação (quaternion)' if has_orientation
                                  else 'integração do giroscópio Z'))

        if has_orientation:
            self.imu_yaw_raw = yaw_from_quaternion(msg.orientation)
        else:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if self.last_gyro_time is not None:
                dt = t - self.last_gyro_time
                if 0.0 < dt < 0.5:  # descarta saltos/reordenação
                    self.imu_yaw_raw = normalize_angle(
                        self.imu_yaw_raw + msg.angular_velocity.z * dt)
            self.last_gyro_time = t

        # Alinha a IMU ao referencial da odometria na primeira amostra
        if self.imu_yaw_offset is None and self.odom_stamp is not None:
            self.imu_yaw_offset = (
                normalize_angle(self.odom_yaw - self.imu_yaw_raw)
                if self.align_imu_to_odom else 0.0)
            self.get_logger().info(
                f'IMU alinhada: offset de yaw = {math.degrees(self.imu_yaw_offset):.1f} deg')

        self.imu_stamp = self.get_clock().now()

    # ===================================================================== #
    # Utilitários
    # ===================================================================== #
    def current_yaw(self) -> float:
        if self.use_imu_heading and self.imu_yaw_offset is not None:
            return normalize_angle(self.imu_yaw_raw + self.imu_yaw_offset)
        return self.odom_yaw

    def data_ok(self) -> bool:
        """Verifica se odom (e IMU, se usada) chegaram e não estão atrasadas."""
        now = self.get_clock().now()

        def fresh(stamp):
            return stamp is not None and \
                (now - stamp).nanoseconds * 1e-9 < self.data_timeout

        if not fresh(self.odom_stamp):
            return False
        if self.use_imu_heading and (not fresh(self.imu_stamp)
                                     or self.imu_yaw_offset is None):
            return False
        return True

    def publish_cmd(self, v: float, w: float):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)

    def publish_status(self, text: str):
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def publish_debug(self, target_heading, heading_error, distance):
        self.target_heading_pub.publish(Float64(data=float(target_heading)))
        self.heading_error_pub.publish(Float64(data=float(heading_error)))
        self.distance_pub.publish(Float64(data=float(distance)))
        self.active_wp_pub.publish(Int32(data=self.current_idx))

        gx, gy = self.waypoints[self.current_idx]
        goal = PointStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.frame_id
        goal.point.x, goal.point.y = gx, gy
        self.goal_pub.publish(goal)

    def advance_waypoint(self, distance: float):
        """Marca a base ativa como alcançada e seleciona a próxima."""
        n = len(self.waypoints)
        name = self.waypoint_names[self.current_idx]
        self.publish_status(
            f'[BASE ALCANÇADA] {self.current_idx + 1}/{n} "{name}" '
            f'(erro final = {distance:.2f} m, pose = ({self.x:.2f}, {self.y:.2f}))')

        self.current_idx += 1
        if self.current_idx >= n:
            if self.loop:
                self.current_idx = 0
            else:
                self.state = NavState.FINISHED
                self.active_wp_pub.publish(Int32(data=-1))
                self.publish_status('[FIM] Todas as bases do losango foram visitadas.')
                return

        self.state = NavState.DWELLING
        self.dwell_until = self.get_clock().now() + Duration(seconds=self.dwell_time)
        nx, ny = self.waypoints[self.current_idx]
        self.publish_status(
            f'[PRÓXIMA BASE] {self.current_idx + 1}/{n} '
            f'"{self.waypoint_names[self.current_idx]}" em ({nx:.2f}, {ny:.2f})')

    # ===================================================================== #
    # Laço de controle
    # ===================================================================== #
    def control_loop(self):
        if self.state == NavState.FINISHED:
            self.publish_cmd(0.0, 0.0)
            return

        # 1) Segurança: sem dados confiáveis -> robô parado
        if not self.data_ok():
            if self.state != NavState.WAITING_DATA:
                self.get_logger().warn('Odometria/IMU indisponível ou atrasada: parando.')
            self.state = NavState.WAITING_DATA
            self.publish_cmd(0.0, 0.0)
            self.get_logger().info('Aguardando odometria e IMU...',
                                   throttle_duration_sec=2.0)
            return
        if self.state == NavState.WAITING_DATA:
            self.state = NavState.DRIVING
            gx, gy = self.waypoints[self.current_idx]
            self.publish_status(
                f'[INÍCIO] Indo para base {self.current_idx + 1}/{len(self.waypoints)} '
                f'"{self.waypoint_names[self.current_idx]}" em ({gx:.2f}, {gy:.2f})')

        # 2) Geometria robô -> base ativa
        yaw = self.current_yaw()
        gx, gy = self.waypoints[self.current_idx]
        dx, dy = gx - self.x, gy - self.y
        distance = math.hypot(dx, dy)                       # distância euclidiana
        target_heading = math.atan2(dy, dx)                 # rumo do vetor direção
        heading_error = normalize_angle(target_heading - yaw)   # delta theta
        self.publish_debug(target_heading, heading_error, distance)

        # 3) Critério de aceitação (raio de tolerância)
        overshot = (distance < 0.70 and math.cos(heading_error) < 0.0)
        if distance <= self.goal_tolerance or overshot:
            self.publish_cmd(0.0, 0.0)
            self.advance_waypoint(distance)
            return

        # 4) Parada sobre a base anterior
        if self.state == NavState.DWELLING:
            if self.get_clock().now() < self.dwell_until:
                self.publish_cmd(0.0, 0.0)
                return
            self.state = NavState.DRIVING

        # 5) Alcançabilidade para Ackermann: a base está dentro do círculo de
        #    giro mínimo (centro do giro = ICR no lado da base)?
        x_r = math.cos(yaw) * dx + math.sin(yaw) * dy       # base no frame do robô
        y_r = -math.sin(yaw) * dx + math.cos(yaw) * dy
        dist_to_icr = math.hypot(x_r, abs(y_r) - self.r_min)
        now = self.get_clock().now()

        if self.state == NavState.DRIVING and dist_to_icr < 0.75 * self.r_min:
            self.state = NavState.REVERSING
            self.reverse_since = now
            self.get_logger().info(
                f'Base dentro do raio de giro (dtheta={math.degrees(heading_error):.0f} deg): '
                'manobrando de ré.')
        elif self.state == NavState.REVERSING:
            reverse_elapsed = (now - self.reverse_since).nanoseconds * 1e-9
            if (dist_to_icr > 1.15 * self.r_min or abs(heading_error) < 0.35
                    or reverse_elapsed > self.max_reverse_time):
                self.state = NavState.DRIVING

        # 6) Comandos
        if self.state == NavState.REVERSING:
            # Ré com esterço que gira o robô em direção à base.
            # Para o Twist, (v<0, w>0) gera yaw rate positivo, logo o sinal de w
            # segue o sinal do erro angular.
            v = -self.v_rev
            w = math.copysign(self.v_rev / self.r_min, heading_error)
        else:
            # Velocidade: reduz perto da base e quando desalinhado
            v = self.v_max * min(1.0, distance / self.r_slow) * max(math.cos(heading_error), 0.0)
            v = clamp(v, self.v_min, self.v_max)
            # Velocidade angular proporcional ao erro, limitada pelo esterço máximo
            w_max = v / self.r_min
            w = clamp(self.k_heading * heading_error, -w_max, w_max)

        self.publish_cmd(v, w)
        self.get_logger().debug(
            f'[{self.state.name}] wp={self.current_idx} rho={distance:.2f} '
            f'dtheta={math.degrees(heading_error):.1f}deg v={v:.2f} w={w:.2f}')

    def stop(self):
        """Envia comando nulo (usado no encerramento)."""
        self.publish_cmd(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
