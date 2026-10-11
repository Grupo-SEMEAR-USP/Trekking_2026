#!/usr/bin/env python3
"""
movement.py - movimentos basicos em malha fechada sobre /odom -> /cmd_vel.

Port do movement.cpp (ROS1) para ROS2 Jazzy, sem threads: uma maquina de
estados rodando em um timer de 20 Hz (o ESP32 para os motores se parar de
receber comandos, entao o /cmd_vel precisa ser publicado continuamente).

Robo Ackermann nao gira sobre o proprio eixo: a curva e feita como ARCO
(v > 0 e w = v / raio). Com turn_radius = 0 vira giro axial (so diferencial/sim).
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MovementNode(Node):

    def __init__(self):
        super().__init__('movement_node')

        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('wait_start', False)      # espera /start_engines
        self.declare_parameter('linear_speed', 0.1)      # m/s
        self.declare_parameter('distance', 2.0)          # m
        self.declare_parameter('turn_deg', 90.0)         # graus (+ = esquerda)
        self.declare_parameter('turn_radius', 1.0)       # m; 0 = giro axial
        self.declare_parameter('turn_speed', 0.2)        # m/s (arco) ou rad/s (axial)
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.1)
        self.declare_parameter('max_correction', 0.5)    # rad/s
        self.declare_parameter('odom_timeout', 0.5)      # s sem odom -> para
        self.declare_parameter('pause_s', 0.5)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.create_subscription(Empty, 'start_engines', self.start_cb, 10)

        self.x = self.y = self.yaw = 0.0
        self.last_odom = None          # rclpy Time da ultima odom
        self.started = not self.get_parameter('wait_start').value
        self.done = False

        p = self.get_parameter
        self.steps = [
            ('pause', p('pause_s').value, 0.0),
            ('straight', p('distance').value, p('linear_speed').value),
            ('pause', p('pause_s').value, 0.0),
            ('turn', math.radians(p('turn_deg').value), p('turn_speed').value),
        ]
        self.idx = -1
        self.step_state = None
        self.last_log = 0.0

        self.dt = 1.0 / p('rate_hz').value
        self.timer = self.create_timer(self.dt, self.loop)
        self.get_logger().info('Aguardando /odom' + (' e /start_engines...' if not self.started else '...'))

    # ---------------- callbacks ----------------
    def start_cb(self, _msg):
        if not self.started:
            self.get_logger().info("'start_engines' recebido, iniciando missao.")
        self.started = True

    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.last_odom = self.get_clock().now()

    # ---------------- helpers ----------------
    def publish(self, v=0.0, w=0.0):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)

    def stop(self):
        for _ in range(3):
            self.publish(0.0, 0.0)

    def odom_ok(self) -> bool:
        if self.last_odom is None:
            return False
        age = (self.get_clock().now() - self.last_odom).nanoseconds * 1e-9
        return age < self.get_parameter('odom_timeout').value

    def log_throttled(self, text, period=0.5):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_log >= period:
            self.last_log = now
            self.get_logger().info(text)

    # ---------------- maquina de estados ----------------
    def begin_step(self):
        kind, value, speed = self.steps[self.idx]
        now = self.get_clock().now().nanoseconds * 1e-9
        self.step_state = {
            'kind': kind, 'value': value, 'speed': speed, 't0': now,
            'x0': self.x, 'y0': self.y, 'yaw0': self.yaw,
            'prev_yaw': self.yaw, 'turned': 0.0,
            'integral': 0.0, 'prev_err': 0.0,
        }
        if kind == 'straight':
            self.get_logger().info(f'Reto: {value:.2f} m a {speed:.2f} m/s')
            self.step_state['timeout'] = abs(value / speed) * 3.0 + 5.0 if speed else 5.0
        elif kind == 'turn':
            self.get_logger().info(f'Giro: {math.degrees(value):.1f} graus')
            self.step_state['timeout'] = 60.0

    def loop(self):
        if self.done or not self.started:
            return
        if not self.odom_ok():
            if self.step_state is not None:      # perdeu odom no meio do movimento
                self.get_logger().warn('Odom ausente/atrasada: parando.', throttle_duration_sec=1.0)
                self.stop()
            return

        if self.step_state is None:
            self.idx += 1
            if self.idx >= len(self.steps):
                self.stop()
                self.get_logger().info('Sequencia finalizada.')
                self.done = True
                return
            self.begin_step()

        s = self.step_state
        now = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now - s['t0']
        finished = False

        if s['kind'] == 'pause':
            self.publish(0.0, 0.0)
            finished = elapsed >= s['value']

        elif s['kind'] == 'straight':
            traveled = math.hypot(self.x - s['x0'], self.y - s['y0'])
            if traveled >= abs(s['value']):
                finished = True
            elif elapsed > s['timeout']:
                self.get_logger().error('Timeout no movimento reto (robo nao andou?). Abortando.')
                self.stop()
                self.done = True
                return
            else:
                err = wrap(s['yaw0'] - self.yaw)
                s['integral'] = max(-1.0, min(1.0, s['integral'] + err * self.dt))
                deriv = (err - s['prev_err']) / self.dt
                s['prev_err'] = err
                p = self.get_parameter
                w = p('kp').value * err + p('ki').value * s['integral'] + p('kd').value * deriv
                m = p('max_correction').value
                w = max(-m, min(m, w))
                v = s['speed']
                if v < 0:                      # re: a correcao de heading inverte
                    w = -w
                self.publish(v, w)
                self.log_throttled(f'Percorrido {traveled:.3f}/{abs(s["value"]):.3f} m')

        elif s['kind'] == 'turn':
            s['turned'] += wrap(self.yaw - s['prev_yaw'])
            s['prev_yaw'] = self.yaw
            target = s['value']
            if abs(s['turned']) >= abs(target):
                finished = True
            elif elapsed > s['timeout']:
                self.get_logger().error('Timeout no giro. Abortando.')
                self.stop()
                self.done = True
                return
            else:
                sign = 1.0 if target > 0 else -1.0
                radius = self.get_parameter('turn_radius').value
                spd = abs(s['speed'])
                if radius > 0.0:               # Ackermann: arco
                    self.publish(spd, sign * spd / radius)
                else:                          # diferencial / sim: axial
                    self.publish(0.0, sign * spd)
                self.log_throttled(f'Giro {math.degrees(abs(s["turned"])):.1f}/'
                                   f'{math.degrees(abs(target)):.1f} graus')

        if finished:
            self.stop()
            self.step_state = None


def main(args=None):
    rclpy.init(args=args)
    node = MovementNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()