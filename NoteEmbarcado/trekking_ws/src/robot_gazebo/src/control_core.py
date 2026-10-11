"""Cálculos e protocolo sem dependência de ROS ou hardware; unidades SI."""
import math
import struct

def ackermann(v, omega, wheelbase, track, radius, steering_limit, wheel_limit):
    if not all(math.isfinite(x) for x in (v, omega, wheelbase, track, radius, steering_limit, wheel_limit)):
        raise ValueError('Comando/geometria não finitos')
    if min(wheelbase, track, radius, steering_limit, wheel_limit) <= 0 or steering_limit >= math.pi / 2:
        raise ValueError('Geometria/limites inválidos')
    if abs(v) < 1e-6:
        return 0., 0., 0.
    delta = max(-steering_limit, min(steering_limit, math.atan(wheelbase * omega / v)))
    actual_omega = v * math.tan(delta) / wheelbase
    left, right = (v - actual_omega * track / 2) / radius, (v + actual_omega * track / 2) / radius
    scale = min(1., wheel_limit / max(abs(left), abs(right)))
    return left * scale, right * scale, delta

def command_frame(left, right, steering_rad, led):
    if not all(math.isfinite(x) for x in (left, right, steering_rad)):
        raise ValueError('Comando não finito')
    payload = struct.pack('<fffB', left, right, steering_rad, int(bool(led)))
    return bytes([0xAA]) + payload + bytes([sum(payload) & 255, 0xBB])

class TelemetryParser:
    """Mantém fragmentos e recupera sincronismo após ruído."""
    def __init__(self):
        self.buffer = bytearray()
    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while self.buffer:
            if self.buffer[0] != 0xAA:
                del self.buffer[0]
                continue
            if len(self.buffer) < 19:
                break
            payload = self.buffer[1:17]
            if self.buffer[18] != 0xBB or self.buffer[17] != (sum(payload) & 255):
                del self.buffer[0]
                continue
            frames.append(struct.unpack('<iiiI', payload))
            del self.buffer[:19]
        return frames

class CommandLease:
    def __init__(self, timeout):
        self.timeout = timeout
        self.values = (0., 0., 0.)
        self.received = None
    def update(self, values, now):
        if len(values) != 3 or not all(math.isfinite(v) for v in values):
            self.clear()
            return False
        self.values, self.received = tuple(values), now
        return True
    def clear(self):
        self.values, self.received = (0., 0., 0.), None
    def get(self, now):
        if self.received is None or not 0 <= now - self.received <= self.timeout:
            return (0., 0., 0.)
        return self.values

class OdometryState:
    def __init__(self):
        self.x = self.y = self.theta = self.vx = self.vth = 0.
        self.timestamp = None
    def update(self, x_um, y_um, theta_mrad, timestamp):
        x, y, theta = x_um / 1e6, y_um / 1e6, theta_mrad / 1e3
        self.vx = self.vth = 0.
        if self.timestamp is not None:
            elapsed = (timestamp - self.timestamp) & 0xFFFFFFFF
            if 0 < elapsed <= 1000:
                dt = elapsed / 1000.
                self.vx = ((x-self.x)*math.cos(self.theta) + (y-self.y)*math.sin(self.theta)) / dt
                self.vth = math.atan2(math.sin(theta-self.theta), math.cos(theta-self.theta)) / dt
        self.x, self.y, self.theta, self.timestamp = x, y, theta, timestamp

def depth_roi(depth, x, y, half_size=2):
    height, width = depth.shape[:2]
    ix, iy = min(width-1, max(0, int(round(x)))), min(height-1, max(0, int(round(y))))
    return depth[max(0,iy-half_size):min(height,iy+half_size+1),
                 max(0,ix-half_size):min(width,ix+half_size+1)]
