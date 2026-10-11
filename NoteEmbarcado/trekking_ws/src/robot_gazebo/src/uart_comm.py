#!/usr/bin/env python3
import math
import time
import serial
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Bool
from robot_interfaces.msg import UARTData, VelocityCommand
from control_core import TelemetryParser, CommandLease, command_frame

class UARTDevice(Node):
    def __init__(self):
        super().__init__('uart_comm')
        defaults = dict(port='/dev/ttyACM0', baud=115200, command_timeout=0.3,
                        telemetry_timeout=0.5, max_wheel_speed=10., max_steering_angle=0.35)
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        self.p = {name: self.get_parameter(name).value for name in defaults}
        for name in ('command_timeout','telemetry_timeout','max_wheel_speed','max_steering_angle'):
            if not math.isfinite(self.p[name]) or self.p[name] <= 0:
                raise ValueError(f'{name} inválido')
        self.lease = CommandLease(self.p['command_timeout'])
        self.parser = TelemetryParser()
        self.port = None
        self.last_telemetry = self.last_stamp = self.telemetry_stamp = None
        self.next_reconnect = 0.
        self.led = False
        self.pub = self.create_publisher(UARTData, 'uart_data', 1)
        self.sub = self.create_subscription(VelocityCommand, 'velocity_command', self.on_command, 1)
        self.led_sub = self.create_subscription(Bool, 'led_command', self.on_led, 1)
        self.clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.02, self.tick, clock=self.clock)
    def on_led(self, msg):
        self.led = bool(msg.data)
    def on_command(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        age = self.get_clock().now().nanoseconds / 1e9 - stamp
        values = (msg.angular_speed_left, msg.angular_speed_right, msg.servo_angle)
        if (stamp <= 0 or not -0.05 <= age <= self.p['command_timeout']
                or not all(math.isfinite(v) for v in values)
                or max(abs(values[0]),abs(values[1])) > self.p['max_wheel_speed'] + 1e-5
                or abs(values[2]) > self.p['max_steering_angle'] + 1e-5):
            self.lease.clear()
            return
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return
        self.last_stamp = stamp
        if self.port is not None and self.last_telemetry is not None:
            self.lease.update(values, time.monotonic())
    def disconnect(self):
        self.lease.clear()
        self.last_telemetry = self.last_stamp = self.telemetry_stamp = None
        self.parser = TelemetryParser()
        if self.port is not None:
            try:
                self.port.close()
            except (OSError, serial.SerialException):
                pass
        self.port = None
        self.next_reconnect = time.monotonic() + 1.
    def tick(self):
        now = time.monotonic()
        try:
            if self.port is None:
                if now < self.next_reconnect:
                    return
                self.port = serial.Serial(self.p['port'], self.p['baud'], timeout=0, write_timeout=0.05)
                self.lease.clear()
                self.port.reset_input_buffer()
                self.get_logger().info('UART conectada; aguardando telemetria e comando novos.')
            data = self.port.read(min(self.port.in_waiting,4096))
            for x,y,z,stamp in self.parser.feed(data):
                # Only a new control sample renews the heartbeat.
                if stamp == self.telemetry_stamp:
                    continue
                self.telemetry_stamp = stamp
                msg = UARTData()
                msg.x,msg.y,msg.z,msg.timestamp = x,y,z,stamp
                self.pub.publish(msg)
                self.last_telemetry = now
            if self.last_telemetry is None or now-self.last_telemetry > self.p['telemetry_timeout']:
                self.lease.clear()
            frame = command_frame(*self.lease.get(now), self.led)
            if self.port.write(frame) != len(frame):
                raise serial.SerialException('Escrita UART parcial')
        except (serial.SerialException,OSError) as error:
            self.get_logger().error(str(error))
            self.disconnect()
    def stop(self):
        self.lease.clear()
        if self.port is not None:
            try:
                self.port.write(command_frame(0.,0.,0.,False))
            except (serial.SerialException,OSError):
                pass
        self.disconnect()

def main(args=None):
    rclpy.init(args=args)
    node = UARTDevice()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
