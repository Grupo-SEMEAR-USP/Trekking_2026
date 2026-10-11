#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import struct

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

        self.ackr_commands = [0.0, 0.0, SERVO_INITIAL_ANGLE]

        self.pub_encoder = self.create_publisher(UARTData, '/uart_data', 10)

        self.sub_velocity = self.create_subscription(
            VelocityData, 
            '/velocity_command', 
            self.callback_vel, 
            10
        )

        self.declare_parameter('port', SERIAL_PORT)
        self.declare_parameter('baud_rate', BAUD_RATE)

        port_to_use = self.get_parameter('port').value
        baud_to_use = int(self.get_parameter('baud_rate').value)

        import os
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
            try:
                self.serial_port.dtr = True
                self.serial_port.rts = False
            except Exception:
                pass
            self.get_logger().info(f"Conexão serial com a ESP32 estabelecida com sucesso!")
        except serial.SerialException as e:
            self.get_logger().error(f"Falha ao abrir porta serial {port_to_use}: {e}")
            self.get_logger().error("Dica: verifique se a ESP32 está plugada e rode: sudo chmod 666 /dev/tty*")
            exit(1)

        self.rx_buffer = bytearray()
        self.first_packet_received = False
        self.timer = self.create_timer(0.02, self.update_loop)

    def calculate_checksum(self, data_bytes):
        return sum(data_bytes) & 0xFF

    def callback_vel(self, msg):

        self.ackr_commands[0] = msg.angular_speed_left
        self.ackr_commands[1] = msg.angular_speed_right
        self.ackr_commands[2] = msg.servo_angle

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

            payload = struct.pack('<fff', self.ackr_commands[0], self.ackr_commands[1], self.ackr_commands[2])

            checksum = self.calculate_checksum(payload)

            frame = struct.pack('B', FRAME_SOF) + payload + struct.pack('B', checksum) + struct.pack('B', FRAME_EOF)
            
            self.serial_port.write(frame)
            
        except Exception as e:
            self.get_logger().error(f"Erro na escrita: {str(e)}")

    def update_loop(self):
        
        self.read_loop()
        self.write_data()

    def destroy_node(self):
        
        self.ackr_commands = [0.0, 0.0, SERVO_INITIAL_ANGLE]
        self.write_data()
        
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