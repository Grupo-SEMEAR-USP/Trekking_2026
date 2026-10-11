
import pyrealsense2 as rs

ctx = rs.context()
devices = ctx.query_devices()

if len(devices) == 0:
    print("Nenhuma RealSense encontrada!")
    exit()

for device in devices:
    print("Camera:", device.get_info(rs.camera_info.name))
    print("USB:", device.get_info(rs.camera_info.usb_type_descriptor))

    for sensor in device.query_sensors():
        nome = sensor.get_info(rs.camera_info.name)
        print(f"\nSensor: {nome}")

        for perfil in sensor.get_stream_profiles():
            tipo = perfil.stream_type()

            if tipo in (rs.stream.accel, rs.stream.gyro):
                print(
                    f"Tipo: {tipo} | "
                    f"Frequencia: {perfil.fps()} Hz | "
                    f"Formato: {perfil.format()}"
                )
