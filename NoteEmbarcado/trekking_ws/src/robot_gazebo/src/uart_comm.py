#!/usr/bin/env python3

import os
import struct
import numpy as np
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from robot_interfaces.msg import UARTData, VelocityData

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

FRAME_SOF = 0xAA
FRAME_EOF = 0xBB

PAYLOAD_SIZE_RX = 16
FRAME_SIZE_RX = 1 + PAYLOAD_SIZE_RX + 1 + 1

SERVO_INITIAL_ANGLE = 90.0


class UARTDevice(Node):
    def __init__(self):
        super().__init__('uart_comm')

        # Declara parâmetros de cinemática do robô (Margarete)
        self.declare_parameter('wheel_base', 0.148245)     # [m] Entre-eixos real
        self.declare_parameter('track_width', 0.1680)       # [m] Bitola traseira
        self.declare_parameter('wheel_radius', 0.033)       # [m] Raio da roda (33 mm)
        self.declare_parameter('servo_center', 90.0)        # [deg] Centro neutro do servo
        self.declare_parameter('max_steer_deg', 30.0)       # [deg] Limite max de esterço
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('port', SERIAL_PORT)
        self.declare_parameter('baud_rate', BAUD_RATE)

        self.L = float(self.get_parameter('wheel_base').value)
        self.W = float(self.get_parameter('track_width').value)
        self.r = float(self.get_parameter('wheel_radius').value)
        self.servo_center = float(self.get_parameter('servo_center').value)
        self.max_steer_deg = float(self.get_parameter('max_steer_deg').value)
        cmd_topic = str(self.get_parameter('cmd_vel_topic').value)

        self.ackr_commands = [0.0, 0.0, self.servo_center]

        # Publicador de odometria dos encoders
        self.pub_encoder = self.create_publisher(UARTData, '/uart_data', 10)

        # 1. Assina /cmd_vel DIRETAMENTE (reduzindo a cadeia de processos)
        self.sub_cmd_vel = self.create_subscription(
            Twist,
            cmd_topic,
            self.callback_cmd_vel,
            10
        )

        # 2. Mantém compatibilidade com /velocity_command legada
        self.sub_velocity = self.create_subscription(
            VelocityData, 
            '/velocity_command', 
            self.callback_vel_legacy, 
            10
        )

        port_to_use = self.get_parameter('port').value
        baud_to_use = int(self.get_parameter('baud_rate').value)

        if not os.path.exists(port_to_use):
            if os.path.exists('/dev/ttyUSB0'):
                self.get_logger().warn(f"Porta {port_to_use} não encontrada. Usando /dev/ttyUSB0 automaticamente!")
                port_to_use = '/dev/ttyUSB0'
            elif os.path.exists('/dev/ttyACM0'):
                self.get_logger().warn(f"Porta {port_to_use} não encontrada. Usando /dev/ttyACM0 automaticamente!")
                port_to_use = '/dev/ttyACM0'

        try:
            self.get_logger().info(f"Conectando à ESP32 em {port_to_use} @ {baud_to_use} baud...")
            self.serial_port = serial.Serial(
                port=port_to_use,
                baudrate=baud_to_use,
                timeout=0.1 
            )
            # Desativa DTR e RTS para evitar que a ESP32 fique travada em Reset ou Bootloader
            try:
                self.serial_port.dtr = False
                self.serial_port.rts = False
            except Exception:
                pass
            self.get_logger().info("Conexão serial com a ESP32 estabelecida com sucesso!")
        except serial.SerialException as e:
            self.get_logger().error(f"Falha ao abrir porta serial {port_to_use}: {e}")
            self.get_logger().error("Dica: verifique se a ESP32 está plugada e rode: sudo chmod 666 /dev/tty*")
            exit(1)

        self.rx_buffer = bytearray()
        self.first_packet_received = False
        self.timer = self.create_timer(0.02, self.update_loop)

        self.get_logger().info(
            f"uart_comm pronto | Ouvindo {cmd_topic} -> converte Ackermann -> envia ESP32 @ {baud_to_use}bps"
        )

    def calculate_checksum(self, data_bytes):
        return sum(data_bytes) & 0xFF

    def callback_cmd_vel(self, msg: Twist):
        """Converte /cmd_vel (Twist) diretamente em velocidades de roda e esterço de servo."""
        v_x = msg.linear.x
        omega = msg.angular.z

        if abs(v_x) < 0.001:
            if abs(omega) > 0.001:
                delta_rad = np.arctan(self.L * omega / 0.1)
            else:
                delta_rad = 0.0
            v_left = 0.0
            v_right = 0.0
        else:
            R = v_x / omega if abs(omega) > 1e-4 else np.inf
            delta_rad = np.arctan(self.L / R)

            v_left = v_x - (omega * self.W / 2.0)
            v_right = v_x + (omega * self.W / 2.0)

        steer_deg = np.degrees(delta_rad)
        steer_deg = float(np.clip(steer_deg, -self.max_steer_deg, self.max_steer_deg))
        servo_angle = self.servo_center + steer_deg

        w_left = float(v_left / self.r)
        w_right = float(v_right / self.r)

        self.ackr_commands[0] = w_left
        self.ackr_commands[1] = w_right
        self.ackr_commands[2] = servo_angle

    def callback_vel_legacy(self, msg: VelocityData):
        """Callback legado para caso algo publique em /velocity_command."""
        self.ackr_commands[0] = float(msg.angular_speed_left)
        self.ackr_commands[1] = float(msg.angular_speed_right)
        self.ackr_commands[2] = float(msg.servo_angle)

    def read_loop(self):
        try:
            if not self.serial_port.is_open:
                return

            in_waiting = self.serial_port.in_waiting
            if in_waiting > 0:
                self.rx_buffer.extend(self.serial_port.read(in_waiting))

            while len(self.rx_buffer) >= FRAME_SIZE_RX:
                try:
                    sof_idx = self.rx_buffer.index(FRAME_SOF)
                except ValueError:
                    self.rx_buffer.clear()
                    break

                if sof_idx > 0:
                    del self.rx_buffer[:sof_idx]

                if len(self.rx_buffer) < FRAME_SIZE_RX:
                    break

                payload = bytes(self.rx_buffer[1:17])
                received_chk = self.rx_buffer[17]
                received_eof = self.rx_buffer[18]

                if received_eof != FRAME_EOF:
                    self.get_logger().warn(f"Erro de EOF: esperado 0x{FRAME_EOF:02X}, recebido 0x{received_eof:02X}")
                    del self.rx_buffer[0]
                    continue

                calculated_chk = self.calculate_checksum(payload)
                if received_chk != calculated_chk:
                    self.get_logger().warn(f"Erro de Checksum: calculado 0x{calculated_chk:02X}, recebido 0x{received_chk:02X}")
                    del self.rx_buffer[0]
                    continue

                # Frame válido!
                del self.rx_buffer[:FRAME_SIZE_RX]
                x, y, z, timestamp = struct.unpack('<iiiI', payload)
                if not self.first_packet_received:
                    self.get_logger().info(f"Primeiro pacote recebido da ESP32! x={x}, y={y}, z={z}, t={timestamp}")
                    self.first_packet_received = True
                self.publish_encoders(x, y, z, timestamp)

        except Exception as e:
            self.get_logger().error(f"Erro na leitura serial: {e}")

    def publish_encoders(self, x, y, z, timestamp):
        msg = UARTData()
        msg.x = x
        msg.y = y
        msg.z = z
        msg.timestamp = timestamp
        self.pub_encoder.publish(msg)

    def write_data(self):
        try:
            # Envia 3 floats (little-endian 32-bit: <fff) correspondendo a data_to_receive_t na ESP32
            payload = struct.pack('<fff', self.ackr_commands[0], self.ackr_commands[1], self.ackr_commands[2])
            checksum = self.calculate_checksum(payload)
            frame = struct.pack('B', FRAME_SOF) + payload + struct.pack('B', checksum) + struct.pack('B', FRAME_EOF)
            self.serial_port.write(frame)
        except Exception as e:
            self.get_logger().error(f"Erro na escrita serial: {str(e)}")

    def update_loop(self):
        self.read_loop()
        self.write_data()

    def destroy_node(self):
        self.ackr_commands = [0.0, 0.0, self.servo_center]
        try:
            self.write_data()
        except Exception:
            pass
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UARTDevice()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()