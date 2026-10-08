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

        try:

            self.serial_port = serial.Serial(
                port=SERIAL_PORT,
                baudrate=BAUD_RATE,
                timeout=0.1 
            )
            
        except serial.SerialException as e:
            self.get_logger().error(f"Falha ao abrir porta serial: {e}")
            exit(1)

        self.timer = self.create_timer(0.02, self.update_loop)

    def calculate_checksum(self, data_bytes):
        return sum(data_bytes) & 0xFF

    def callback_vel(self, msg):

        self.ackr_commands[0] = msg.angular_speed_left
        self.ackr_commands[1] = msg.angular_speed_right
        self.ackr_commands[2] = msg.servo_angle

    def read_loop(self):

        expected_sof = struct.pack('B', FRAME_SOF)
        expected_eof = struct.pack('B', FRAME_EOF)

        try:

            if self.serial_port.is_open and self.serial_port.in_waiting > 0:
                byte = self.serial_port.read(1)
                
                if byte == expected_sof:
                    rest_of_frame = self.serial_port.read(FRAME_SIZE_RX - 1)
                    
                    if len(rest_of_frame) == (FRAME_SIZE_RX - 1):

                        payload = rest_of_frame[0:16]
                        received_chk = rest_of_frame[16:17]
                        received_eof = rest_of_frame[17:18]
                        
                        if received_eof != expected_eof:
                            self.get_logger().warn("Erro de EOF na serial")
                            return

                        calculated_chk_int = self.calculate_checksum(payload)
                        calculated_chk_byte = struct.pack('B', calculated_chk_int)

                        if received_chk != calculated_chk_byte:
                            self.get_logger().warn("Erro de Checksum na serial")
                            return

                        x, y, z, timestamp = struct.unpack('<iiiI', payload)
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