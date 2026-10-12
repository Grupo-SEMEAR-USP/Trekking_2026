"""Webcam OpenCV, captura continua com somente o frame mais recente."""
import os
import threading
import time


class Webcam:
    def __init__(self, index=0, width=640, height=480, backend='auto'):
        self.index, self.width, self.height = index, width, height
        self.backend = backend
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._packet = None
        self._error = ''
        self._last_seq = 0
        self._thread = threading.Thread(target=self._run, daemon=True, name='webcam-latest')
        self._thread.start()
        try:
            self.read(timeout=8)
        except BaseException:
            self.close()
            raise

    def _run(self):
        import cv2
        backends = ([('dshow' if os.name == 'nt' else 'any'), 'msmf'] if self.backend == 'auto'
                    else [self.backend])
        cap = None
        try:
            for backend in backends:
                api = {'dshow': cv2.CAP_DSHOW, 'msmf': cv2.CAP_MSMF, 'any': cv2.CAP_ANY}[backend]
                cap = cv2.VideoCapture(self.index, api)
                if not cap.isOpened():
                    cap.release()
                    cap = None
                    continue
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                cap.set(cv2.CAP_PROP_FPS, 30)
                ok, frame = cap.read()
                if ok and frame is not None and frame.size:
                    self.backend = backend
                    print(f'Webcam indice {self.index} | {backend} | imagem real {frame.shape[1]}x{frame.shape[0]}')
                    break
                cap.release()
                cap = None
            if cap is None:
                raise RuntimeError('Nao foi possivel abrir webcam. Feche Camera/Teams/Viewer; tente --source 1 ou --backend msmf.')
            seq = 0
            while not self._stop.is_set():
                acquired = time.monotonic()
                seq += 1
                with self._lock:
                    self._packet = (seq, frame.copy(), acquired)
                ok, frame = cap.read()
                if not ok or frame is None or not frame.size:
                    raise RuntimeError('A webcam parou de entregar imagens')
        except Exception as e:
            with self._lock:
                self._error = str(e)
        finally:
            if cap is not None:
                cap.release()

    def read(self, timeout=.25):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            with self._lock:
                packet, error = self._packet, self._error
            if error:
                raise RuntimeError(error)
            if packet and packet[0] != self._last_seq:
                self._last_seq = packet[0]
                return packet[1], None, packet[2]
            time.sleep(.003)
        raise RuntimeError('Webcam sem frame novo; captura interrompida')

    def close(self):
        self._stop.set()
        self._thread.join(timeout=2)
