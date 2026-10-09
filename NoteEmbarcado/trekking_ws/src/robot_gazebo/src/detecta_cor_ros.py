import cv2 as cv
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class DetectorAmarelo(Node):

    def __init__(self):
        super().__init__('detector_amarelo')

        # Publisher ROS 2
        self.publisher = self.create_publisher(
            Bool,
            '/amarelo_detectado',
            10
        )

        # Inicializa a câmera
        self.cap = cv.VideoCapture(0)

        if not self.cap.isOpened():
            raise RuntimeError("Erro ao abrir a câmera!")

        # Variável de controle
        self.amarelo_detectado = False

        # Intervalo HSV da cor amarela
        self.amarelo_min = np.array([20, 100, 100])
        self.amarelo_max = np.array([35, 255, 255])

        # Timer de processamento (aprox. 30 Hz)
        self.timer = self.create_timer(
            1.0 / 30.0,
            self.detectar_amarelo
        )

        self.get_logger().info(
            "Detector de amarelo iniciado!"
        )

    def detectar_amarelo(self):

        ret, frame = self.cap.read()

        if not ret:
            self.amarelo_detectado = False
            self.publicar_estado()
            return

        # Converte BGR para HSV
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        # Cria a máscara binária
        mascara = cv.inRange(
            hsv,
            self.amarelo_min,
            self.amarelo_max
        )

        # Encontra os contornos
        contornos, _ = cv.findContours(
            mascara,
            cv.RETR_EXTERNAL,
            cv.CHAIN_APPROX_SIMPLE
        )

        # Reinicia a variável a cada frame
        self.amarelo_detectado = False

        for contorno in contornos:

            # Ignora regiões pequenas
            if cv.contourArea(contorno) > 150000:

                # Atualiza a variável de controle
                self.amarelo_detectado = True

                # Calcula o retângulo delimitador
                x, y, w, h = cv.boundingRect(contorno)

                # Desenha o retângulo
                cv.rectangle(
                    frame,
                    (x, y),
                    (x + w, y + h),
                    (0, 255, 0),
                    2
                )

                # Calcula o centro do objeto
                centro_x = x + w // 2
                centro_y = y + h // 2

                # Marca o centro
                cv.circle(
                    frame,
                    (centro_x, centro_y),
                    5,
                    (0, 0, 255),
                    -1
                )

                # Identifica o objeto
                cv.putText(
                    frame,
                    "Amarelo",
                    (x, y - 10),
                    cv.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )

        # Publica o estado no ROS 2
        self.publicar_estado()

        # Exibe o estado na imagem
        cv.putText(
            frame,
            f"Amarelo detectado: {self.amarelo_detectado}",
            (20, 30),
            cv.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0) if self.amarelo_detectado else (0, 0, 255),
            2
        )

        # Exibe imagem e máscara
        cv.imshow("Deteccao de Amarelo", frame)
        cv.imshow("Mascara", mascara)

        # Pressione Q para sair
        if cv.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    def publicar_estado(self):

        msg = Bool()
        msg.data = self.amarelo_detectado

        self.publisher.publish(msg)

    def destroy_node(self):

        self.cap.release()
        cv.destroyAllWindows()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = None

    try:
        node = DetectorAmarelo()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()