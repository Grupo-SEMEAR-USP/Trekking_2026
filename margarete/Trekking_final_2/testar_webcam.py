"""Abre somente a webcam; sem YOLO, ESP32 ou gravacao de imagens."""
import argparse
import cv2
from camera_webcam import Webcam


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', type=int, default=0)
    p.add_argument('--backend', choices=['auto','dshow','msmf','any'], default='auto')
    p.add_argument('--check', action='store_true', help='Captura dez frames, sem abrir janela')
    a = p.parse_args()
    camera = Webcam(a.source, backend=a.backend)
    try:
        count=0
        while True:
            image, _, _ = camera.read()
            if a.check:
                count += 1
                if count == 10:
                    print('OK: dez frames novos; nenhum arquivo de imagem salvo.')
                    break
            else:
                cv2.imshow('Webcam - Q para sair', image)
                if cv2.waitKey(1) & 0xff in (ord('q'),27): break
    finally:
        camera.close()
        cv2.destroyAllWindows()


if __name__ == '__main__': main()
