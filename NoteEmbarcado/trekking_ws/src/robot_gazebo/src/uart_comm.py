```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import serial
import struct

from robot_interfaces.msg import UARTData, VelocityData


# Configuracao da UART
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

# Delimitadores dos frames
FRAME_SOF = 0xAA
FRAME_EOF = 0xBB

# Payload recebido da ESP32:
# x, y, z: 3 inteiros de 32 bits
# timestamp: 1 inteiro sem sinal de 32 bits
PAYLOAD_SIZE_RX = 16
FRAME_SIZE_RX = 1 + PAYLOAD_SIZE_RX + 1 + 1

# Payload enviado para a ESP32:
# 3 floats + 1 byte para o LED
PAYLOAD_SIZE_TX = 13
FRAME_SIZE_TX = 1 + PAYLOAD_SIZE_TX + 1 + 1

SERVO_INITIAL_ANGLE = 90.0


class UARTDevice(Node):

    def __init__(self):

        super().__init__('uart_comm')

        # Comandos Ackermann:
        # [velocidade esquerda, velocidade direita, angulo servo]
        self.ackr_commands = [
            0.0,
            0.0,
            SERVO_INITIAL_ANGLE
        ]

        # Estado inicial do LED: desligado
        self.led_status = False

        # Publisher: dados recebidos da ESP32
        self.pub_encoder = self.create_publisher(
            UARTData,
            '/uart_data',
            10
        )

        # Subscriber: comandos de velocidade
        self.sub_velocity = self.create_subscription(
            VelocityData,
            '/velocity_command',
            self.callback_vel,
            10
        )

        # Subscriber: deteccao de amarelo
        self.sub_visao = self.create_subscription(
            Bool,
            '/amarelo_detectado',
            self.callback_visao,
            10
        )

        # Inicializacao da comunicacao serial
        try:
            self.serial_port = serial.Serial(
                port=SERIAL_PORT,
                baudrate=BAUD_RATE,
                timeout=0.1
            )

        except serial.SerialException as e:
            self.get_logger().error(
                f"Falha ao abrir porta serial: {e}"
            )
            raise

        # Loop de comunicacao a cada 20 ms (50 Hz)
        self.timer = self.create_timer(
            0.02,
            self.update_loop
        )

        self.get_logger().info(
            "Comunicacao UART iniciada!"
        )

    def calculate_checksum(self, data_bytes):
        """
        Calcula o checksum pela soma dos bytes,
        mantendo somente os 8 bits menos significativos.
        """
        return sum(data_bytes) & 0xFF

    def callback_visao(self, msg):
        """
        Atualiza o estado do LED conforme a deteccao de amarelo.
        True = ligado; False = desligado.
        """
        self.led_status = msg.data

    def callback_vel(self, msg):
        """
        Atualiza os tres comandos Ackermann recebidos pelo ROS 2.
        """
        self.ackr_commands[0] = msg.angular_speed_left
        self.ackr_commands[1] = msg.angular_speed_right
        self.ackr_commands[2] = msg.servo_angle

    def read_loop(self):
        """
        Recebe frames da ESP32.

        Formato:
        SOF + payload de 16 bytes + checksum + EOF
        """

        expected_sof = struct.pack('B', FRAME_SOF)
        expected_eof = struct.pack('B', FRAME_EOF)

        try:

            if not self.serial_port.is_open:
                return

            if self.serial_port.in_waiting == 0:
                return

            # Procura o inicio do frame
            byte = self.serial_port.read(1)

            if byte != expected_sof:
                return

            # Le o restante do frame
            rest_of_frame = self.serial_port.read(
                FRAME_SIZE_RX - 1
            )

            if len(rest_of_frame) != FRAME_SIZE_RX - 1:
                self.get_logger().warning(
                    "Frame UART incompleto"
                )
                return

            # Separa payload, checksum e EOF
            payload = rest_of_frame[:PAYLOAD_SIZE_RX]

            received_chk = rest_of_frame[
                PAYLOAD_SIZE_RX:PAYLOAD_SIZE_RX + 1
            ]

            received_eof = rest_of_frame[
                PAYLOAD_SIZE_RX + 1:PAYLOAD_SIZE_RX + 2
            ]

            # Verifica EOF
            if received_eof != expected_eof:
                self.get_logger().warning(
                    "Erro de EOF na serial"
                )
                return

            # Verifica checksum
            calculated_chk = self.calculate_checksum(payload)

            calculated_chk_byte = struct.pack(
                'B',
                calculated_chk
            )

            if received_chk != calculated_chk_byte:
                self.get_logger().warning(
                    "Erro de Checksum na serial"
                )
                return

            # Interpreta os dados recebidos
            x, y, z, timestamp = struct.unpack(
                '<iiiI',
                payload
            )

            self.publish_encoders(
                x,
                y,
                z,
                timestamp
            )

        except (serial.SerialException, OSError) as e:
            self.get_logger().error(
                f"Erro na leitura serial: {e}"
            )

    def publish_encoders(self, x, y, z, timestamp):
        """
        Publica os dados recebidos da ESP32 no ROS 2.
        """

        msg = UARTData()

        msg.x = x
        msg.y = y
        msg.z = z
        msg.timestamp = timestamp

        self.pub_encoder.publish(msg)

    def write_data(self):

        try:

            if not self.serial_port.is_open:
                return

            # Converte o estado do LED em um byte
            led_byte = 1 if self.led_status else 0

            # Monta o payload:
            # 0-3:   velocidade angular esquerda
            # 4-7:   velocidade angular direita
            # 8-11:  angulo do servo
            # 12:    estado do LED
            payload = struct.pack(
                '<fffB',
                self.ackr_commands[0],
                self.ackr_commands[1],
                self.ackr_commands[2],
                led_byte
            )

            # Verifica o tamanho do payload
            if len(payload) != PAYLOAD_SIZE_TX:
                self.get_logger().error(
                    "Tamanho incorreto do payload TX"
                )
                return

            # Calcula checksum sobre os 13 bytes do payload
            checksum = self.calculate_checksum(payload)

            # Monta o frame UART completo
            frame = (
                struct.pack('B', FRAME_SOF)
                + payload
                + struct.pack('B', checksum)
                + struct.pack('B', FRAME_EOF)
            )

            # Envia o frame para a ESP32
            self.serial_port.write(frame)

        except (serial.SerialException, OSError) as e:
            self.get_logger().error(
                f"Erro na escrita serial: {e}"
            )

    def update_loop(self):

        # Recebe dados da ESP32
        self.read_loop()

        # Envia comandos para a ESP32
        self.write_data()

    def destroy_node(self):
        """
        Tenta enviar comandos seguros antes de encerrar.
        """

        # Zera as velocidades e retorna o servo a 90 graus
        self.ackr_commands = [
            0.0,
            0.0,
            SERVO_INITIAL_ANGLE
        ]

        # Solicita o desligamento do LED
        self.led_status = False

        # Envia os comandos finais antes de fechar a serial
        if (
            hasattr(self, 'serial_port')
            and self.serial_port.is_open
        ):
            try:
                self.write_data()
                self.serial_port.close()

            except (serial.SerialException, OSError) as e:
                self.get_logger().error(
                    f"Erro ao encerrar a UART: {e}"
                )

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = None

    try:
        node = UARTDevice()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except (RuntimeError, serial.SerialException) as e:
        print(f"Erro ao iniciar uart_comm: {e}")

    finally:

        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
```
