
import pyrealsense2 as rs
import numpy as np
import cv2

ctx = rs.context()
devices = ctx.query_devices()

if len(devices) == 0:
    raise RuntimeError("Nenhuma RealSense encontrada")

device = devices[0]

# Descobre os modos RGB disponíveis
perfis = []

for sensor in device.query_sensors():
    for perfil in sensor.get_stream_profiles():
        if perfil.stream_type() != rs.stream.color:
            continue

        if perfil.format() not in (rs.format.bgr8, rs.format.rgb8):
            continue

        video = perfil.as_video_stream_profile()
        perfis.append((
            video.width(),
            video.height(),
            perfil.fps(),
            perfil.format()
        ))

# Prioriza menor largura de banda
perfis.sort(key=lambda p: p[0] * p[1] * p[2])

if not perfis:
    raise RuntimeError("Nenhum perfil RGB compatível encontrado")

for largura, altura, fps, formato in perfis:
    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(
        rs.stream.color,
        largura,
        altura,
        formato,
        fps
    )

    iniciou = False
    recebeu_frame = False

    try:
        print(f"Testando {largura}x{altura} @ {fps} FPS")

        pipeline.start(config)
        iniciou = True

        # Aguarda até 10 segundos por um frame
        frames = pipeline.wait_for_frames(10000)
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        recebeu_frame = True
        print("Captura funcionando!")

        while True:
            frames = pipeline.wait_for_frames(10000)
            color_frame = frames.get_color_frame()

            if not color_frame:
                continue

            imagem = np.asanyarray(color_frame.get_data())

            if formato == rs.format.rgb8:
                imagem = cv2.cvtColor(imagem, cv2.COLOR_RGB2BGR)

            cv2.imshow("RealSense D435i", imagem)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except RuntimeError as erro:
        print("Falhou:", erro)

    finally:
        if iniciou:
            pipeline.stop()
        cv2.destroyAllWindows()

    if recebeu_frame:
        break
else:
    print("Nenhum dos perfis testados conseguiu receber frames.")
