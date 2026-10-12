"""Protocolo T1 proprio do teste; NAO compativel com firmware antigo."""
import binascii
import math
import time


def pack(body):
    raw = body.encode('ascii')
    return raw + f'*{binascii.crc_hqx(raw, 0xffff):04X}\n'.encode('ascii')


def unpack(line):
    raw, checksum = line.strip().rsplit(b'*', 1)
    if len(checksum) != 4 or binascii.crc_hqx(raw, 0xffff) != int(checksum, 16):
        raise ValueError('CRC incorreto')
    return raw.decode('ascii')


class SerialTransport:
    def __init__(self, port):
        import serial
        self.serial = serial.Serial(port, 115200, timeout=.015, write_timeout=.1)
        self.seq = 0
        # Abrir USB pode reiniciar a ESP32. Nenhum comando de movimento aqui.
        time.sleep(2)
        self.serial.reset_input_buffer()
        try:
            self.send('STOP')  # handshake ACK identifica o firmware T1
        except BaseException:
            self.serial.close()
            raise

    def send(self, mode, throttle=0., steering=0.):
        if mode not in ('STOP', 'ARM', 'DRIVE'):
            raise ValueError('Modo invalido')
        if not all(math.isfinite(x) for x in (throttle, steering)):
            raise ValueError('Comando nao finito')
        if not 0 <= throttle <= .25 or not -1 <= steering <= 1:
            raise ValueError('Comando fora dos limites')
        if mode != 'DRIVE' and (throttle != 0 or steering != 0):
            raise ValueError('ARM e STOP devem ser zero')
        self.seq += 1
        self.serial.write(pack(f'T1 {self.seq} {mode} {throttle:.4f} {steering:.4f}'))
        deadline = time.monotonic() + .15
        expected = 'ARMED' if mode in ('ARM', 'DRIVE') else 'STOPPED'
        while time.monotonic() < deadline:
            line = self.serial.readline(128)
            try:
                parts = unpack(line).split()
            except (ValueError, UnicodeError):
                continue
            if len(parts) == 4 and parts[:3] == ['ACK', 'T1', str(self.seq)]:
                if parts[3] != expected:
                    raise RuntimeError(f'ESP32 recusou comando: {parts[3]}')
                return
        raise TimeoutError('ESP32 sem ACK T1; movimento interrompido')

    def close(self):
        try:
            self.send('STOP')
        finally:
            self.serial.close()
