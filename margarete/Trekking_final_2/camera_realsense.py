import time
import numpy as np


def start_supported_pipeline(rs):
    """Consulta perfis reais e valida combinacao RGB/Z16 antes de iniciar."""
    context = rs.context()
    devices = list(context.query_devices())
    if len(devices) != 1:
        raise RuntimeError(f'Encontradas {len(devices)} RealSense. Conecte apenas a D435i e feche o Viewer.')
    device = devices[0]
    name = device.get_info(rs.camera_info.name)
    usb = device.get_info(rs.camera_info.usb_type_descriptor) if device.supports(rs.camera_info.usb_type_descriptor) else '?'
    print(f'Camera: {name} | conexao USB: {usb}')
    if usb.startswith('2'):
        print('AVISO: conexao USB 2 detectada. Prefira cabo e porta USB 3, sem hub.')
    available = set()
    for sensor in device.sensors:
        for p in sensor.get_stream_profiles():
            if p.is_video_stream_profile():
                v = p.as_video_stream_profile()
                available.add((p.stream_type(), v.width(), v.height(), p.format(), p.fps()))
    serial = device.get_info(rs.camera_info.serial_number)
    # Perfis de profundidade pequenos primeiro, para validar distancias curtas.
    # Em USB 2 a D435i consultada nao anuncia depth 424x240.
    candidates = [
        (424, 240, 640, 480, 30), (480, 270, 640, 480, 30),
        (640, 480, 640, 480, 30), (480, 270, 424, 240, 30),
        (480, 270, 640, 480, 15), (640, 480, 640, 480, 15),
        (480, 270, 424, 240, 15),
    ]
    if usb.startswith('2'):
        # USB 2 pode anunciar 30 fps sem conseguir entregar RGB + Z16 juntos.
        candidates.sort(key=lambda p: p[4] != 15)
    errors = []
    for dw, dh, cw, ch, fps in candidates:
        if ((rs.stream.depth, dw, dh, rs.format.z16, fps) not in available or
                (rs.stream.color, cw, ch, rs.format.bgr8, fps) not in available):
            continue
        pipeline = rs.pipeline(context)
        config = rs.config()
        config.enable_device(serial)
        config.enable_stream(rs.stream.depth, dw, dh, rs.format.z16, fps)
        config.enable_stream(rs.stream.color, cw, ch, rs.format.bgr8, fps)
        if not config.can_resolve(rs.pipeline_wrapper(pipeline)):
            continue
        try:
            profile = pipeline.start(config)
            pipeline.wait_for_frames(5000)
        except RuntimeError as e:
            errors.append(f'{dw}x{dh}/{cw}x{ch}@{fps}: {e}')
            # Um start parcialmente iniciado pode precisar liberar sensores.
            try:
                pipeline.stop()
            except RuntimeError:
                pass
            continue
        print(f'Perfil ativo: profundidade {dw}x{dh}; RGB {cw}x{ch}; {fps} fps')
        return pipeline, profile
    detail = '\n'.join(errors)
    raise RuntimeError('Nao foi possivel iniciar RGB + profundidade. Feche o RealSense Viewer, '
                       'reconecte a camera em USB 3 e confira o cabo.\n'+detail)


class RealSense:
    def __init__(self):
        import pyrealsense2 as rs
        self.rs = rs
        self.pipeline, profile = start_supported_pipeline(rs)
        try:
            for sensor in profile.get_device().sensors:
                if sensor.supports(rs.option.global_time_enabled):
                    sensor.set_option(rs.option.global_time_enabled, 1)
            self.scale = profile.get_device().first_depth_sensor().get_depth_scale()
            self.align = rs.align(rs.stream.color)
        except BaseException:
            self.pipeline.stop()
            raise
        self.last_number = None

    def read(self):
        # Drena frames disponiveis; nao avanca usando fila antiga.
        frames = self.pipeline.wait_for_frames(500)
        for _ in range(8):
            newer = self.pipeline.poll_for_frames()
            if not newer:
                break
            frames = newer
        acquired = time.monotonic()
        color_native = frames.get_color_frame()
        if not color_native:
            raise RuntimeError('RealSense sem imagem colorida')
        number = color_native.get_frame_number()
        if number == self.last_number:
            raise RuntimeError('Frame repetido')
        self.last_number = number
        # Quando timestamp global/system esta disponivel, verifica fila no SDK.
        domain = color_native.get_frame_timestamp_domain()
        if domain in (self.rs.timestamp_domain.system_time, self.rs.timestamp_domain.global_time):
            if abs(time.time()-color_native.get_timestamp()/1000) > .25:
                raise RuntimeError('Frame antigo no SDK')
        aligned = self.align.process(frames)
        color, depth = aligned.get_color_frame(), aligned.get_depth_frame()
        if not color or not depth:
            raise RuntimeError('RealSense sem RGB/profundidade alinhados')
        return np.asanyarray(color.get_data()).copy(), np.asanyarray(depth.get_data()).astype(np.float32)*self.scale, acquired

    def close(self):
        self.pipeline.stop()
