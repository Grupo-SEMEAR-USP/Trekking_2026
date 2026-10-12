"""Modo padrao: visualizacao. USB somente com --live --port COMx."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import time
from control import Target, TargetLock, command, depth_for_cone, validate_config


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--live', action='store_true')
    p.add_argument('--port')
    p.add_argument('--camera', choices=['webcam','realsense'], default='webcam')
    p.add_argument('--source', type=int, help='Indice da webcam, padrao 0')
    p.add_argument('--backend', choices=['auto','dshow','msmf','any'])
    p.add_argument('--calibrate-distance', type=float, help='Distancia medida da lente ao cone, em metros; C salva')
    p.add_argument('--config')
    args = p.parse_args()
    root = Path(__file__).resolve().parent
    config_path = Path(args.config) if args.config else root / ('config_webcam.json' if args.camera == 'webcam' else 'config.json')
    cfg = json.loads(config_path.read_text(encoding='utf-8'))
    validate_config(cfg)
    if args.calibrate_distance is not None and (args.camera != 'webcam' or args.live or not .3 <= args.calibrate_distance <= 2):
        p.error('Calibracao apenas visual com webcam; distancia entre 0.30 e 2 m')
    if args.live and (not args.port or not cfg['hardware_calibrated']):
        p.error('--live exige --port e hardware_calibrated=true apos calibracao manual')
    import cv2
    from ultralytics import YOLO
    from webcam_distance import CalibrationSamples, estimate_distance
    from transport_serial import SerialTransport
    model_path = root / cfg['model']
    if not model_path.is_file():
        raise FileNotFoundError(model_path)
    model_hash = hashlib.sha256(model_path.read_bytes()).hexdigest()
    calibration_path = root / 'webcam_calibration.json'
    calibration = json.loads(calibration_path.read_text(encoding='utf-8')) if calibration_path.exists() and args.camera == 'webcam' else None
    source = args.source if args.source is not None else cfg.get('webcam_source',0)
    if source < 0:
        p.error('Indice da webcam deve ser >= 0')
    if args.live and args.camera == 'webcam' and (not calibration or cfg.get('webcam_distance_validated') is not True):
        p.error('Webcam --live exige calibracao e webcam_distance_validated=true apos verificar distancia e montagem')
    model = YOLO(str(model_path))
    print('Classes do modelo:', model.names)
    if cfg['cone_class'] not in model.names:
        raise ValueError('cone_class nao existe no modelo')
    # Aquecer antes de abrir USB; nenhum comando durante carregamento do YOLO.
    import numpy as np
    model.predict(np.zeros((480, 640, 3), np.uint8), device=cfg['device'], verbose=False)
    camera = transport = None
    root.joinpath('logs').mkdir(exist_ok=True)
    logfile = root / 'logs' / (time.strftime('%Y%m%d_%H%M%S')+'.csv')
    armed = False
    lock = TargetLock()
    samples = CalibrationSamples()
    try:
        if args.camera == 'webcam':
            from camera_webcam import Webcam
            camera = Webcam(source, cfg.get('webcam_width',640), cfg.get('webcam_height',480),
                            args.backend or cfg.get('webcam_backend','auto'))
        else:
            from camera_realsense import RealSense
            camera = RealSense()
        if args.live:
            transport = SerialTransport(args.port)
        with logfile.open('w', newline='', encoding='utf-8') as f:
            log = csv.writer(f)
            log.writerow(['time', 'state', 'camera_m', 'front_m', 'throttle', 'steering', 'frame_age_s', 'armed'])
            while True:
                image, depth, acquired = camera.read()
                result = model.predict(image, conf=cfg['confidence'], classes=[cfg['cone_class']],
                                       device=cfg['device'], imgsz=640, verbose=False)[0]
                candidates = [Target(tuple(map(float, b.xyxy[0].cpu().tolist())), float(b.conf[0])) for b in result.boxes]
                target = lock.update(candidates, image.shape[1])
                age = time.monotonic()-acquired
                if args.camera == 'webcam':
                    distance = estimate_distance(calibration, target, image.shape, source, camera.backend, model_hash)
                    roi = None
                    samples.update(target if len(candidates)==1 else None, image.shape, age)
                else:
                    distance, roi = depth_for_cone(depth, target.box) if target else (None, None)
                state, throttle, steering = command(cfg, target, distance, image.shape[1], age)
                if args.camera == 'webcam' and target and distance is None and age <= cfg['max_frame_age_s']:
                    state = 'SEM_DISTANCIA_VALIDA'
                    error = (target.cx-image.shape[1]/2)/(image.shape[1]/2)
                    steering = max(-1.,min(1.,error*cfg['steering_gain']*cfg['steering_sign']))
                # Falha ou conclusao desarma; so a tecla ESPACO permite novo inicio.
                if armed and state != 'APROXIMANDO':
                    armed = False
                    if transport:
                        transport.send('STOP')
                if armed and transport:
                    transport.send('DRIVE', throttle, steering)
                preview = result.plot()
                if roi:
                    cv2.rectangle(preview, roi[:2], roi[2:], (255, 0, 255), 2)
                front = distance-cfg['camera_recess_m'] if distance is not None else None
                kind = 'frente ESTIMADA' if args.camera == 'webcam' else 'frente'
                label = f'{state} | {kind} {front:.3f} m' if front is not None else state
                cv2.putText(preview, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 255, 255), 2)
                cv2.putText(preview, f'PWM {throttle:.3f} servo {steering:+.2f} | '+('ARMADO' if armed else 'PARADO'),
                            (8, 48), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 255), 1)
                if args.calibrate_distance is not None:
                    cv2.putText(preview, f'C: calibrar a {args.calibrate_distance:.2f}m | amostras {len(samples.samples)}/20',
                                (8,72), cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,255),1)
                cv2.putText(preview, 'ESPACO: iniciar | S: parar | Q: sair | '+('USB' if args.live else 'SIMULACAO'),
                            (8, preview.shape[0]-10), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 255, 255), 1)
                cv2.imshow('Trekking - cone', preview)
                log.writerow([time.time(), state, distance, front, throttle, steering, age, armed])
                f.flush()
                key = cv2.waitKey(1) & 0xff
                if key in (ord('q'), 27):
                    break
                if key == ord('s'):
                    armed = False
                    if transport:
                        transport.send('STOP')
                    lock = TargetLock()
                if key == ord('c') and args.calibrate_distance is not None:
                    try:
                        calibration = samples.calibration(args.calibrate_distance, image.shape, source, camera.backend, model_hash)
                        calibration_path.write_text(json.dumps(calibration,indent=2),encoding='utf-8')
                        print('Calibracao salva:',calibration_path)
                    except ValueError as e:
                        print('Calibracao nao salva:',e)
                if key == ord(' ') and not armed and state == 'APROXIMANDO':
                    if transport:
                        transport.send('ARM')
                    armed = True
    finally:
        # Parar motores antes de fechar camera/janela.
        try:
            if transport:
                transport.close()
        finally:
            if camera:
                camera.close()
            cv2.destroyAllWindows()
    print('Log:', logfile)


if __name__ == '__main__':
    main()
