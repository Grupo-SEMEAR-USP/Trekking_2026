
import pyrealsense2 as rs
import time
import threading

pipeline = rs.pipeline()
config = rs.config()

# Configura os sensores para 200 Hz
config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)
config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)

lock = threading.Lock()

aceleracao = None
giroscopio = None


def receber_frame(frame):
    global aceleracao, giroscopio

    if not frame.is_motion_frame():
        return

    tipo = frame.get_profile().stream_type()
    dados = frame.as_motion_frame().get_motion_data()

    with lock:
        if tipo == rs.stream.accel:
            aceleracao = (dados.x, dados.y, dados.z)

        elif tipo == rs.stream.gyro:
            giroscopio = (dados.x, dados.y, dados.z)


iniciou = False

try:
    pipeline.start(config, receber_frame)
    iniciou = True

    print("RealSense D435i - IMU iniciada!")
    print("Pressione Ctrl+C para sair.\n")

    while True:
        with lock:
            a = aceleracao
            g = giroscopio

        if a is not None:
            print(
                f"ACELERACAO (m/s²): "
                f"X={a[0]:7.3f} | "
                f"Y={a[1]:7.3f} | "
                f"Z={a[2]:7.3f}"
            )

        if g is not None:
            print(
                f"GIROSCOPIO (rad/s): "
                f"X={g[0]:7.3f} | "
                f"Y={g[1]:7.3f} | "
                f"Z={g[2]:7.3f}"
            )

        print("-" * 65)
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nEncerrando...")

except RuntimeError as erro:
    print("Erro:", erro)

finally:
    if iniciou:
        pipeline.stop()
