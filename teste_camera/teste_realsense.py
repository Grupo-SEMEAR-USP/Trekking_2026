
import pyrealsense2 as rs

ctx = rs.context()
dispositivos = ctx.query_devices()

for dispositivo in dispositivos:
    print("Camera:", dispositivo.get_info(rs.camera_info.name))
    print("USB:", dispositivo.get_info(rs.camera_info.usb_type_descriptor))

    for sensor in dispositivo.query_sensors():
        print("\nSensor:", sensor.get_info(rs.camera_info.name))

        for perfil in sensor.get_stream_profiles():
            if perfil.stream_type() == rs.stream.color:
                video = perfil.as_video_stream_profile()
                print(
                    f"{video.width()}x{video.height()} "
                    f"| {perfil.fps()} FPS "
                    f"| {perfil.format()}"
                )
