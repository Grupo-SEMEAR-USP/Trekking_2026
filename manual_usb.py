"""Teste curto de motores/servo. Usar primeiro com rodas suspensas."""
import argparse
import time
from transport_serial import SerialTransport


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', required=True)
    p.add_argument('--throttle', type=float, default=0.)
    p.add_argument('--steering', type=float, default=0.)
    p.add_argument('--seconds', type=float, default=1.)
    a = p.parse_args()
    if not 0 <= a.throttle <= .25 or not -1 <= a.steering <= 1 or not 0 < a.seconds <= 3:
        p.error('throttle 0..0.25; steering -1..1; seconds >0 e <=3')
    # NaN rejeitado pelas comparacoes acima.
    t = SerialTransport(a.port)
    try:
        t.send('ARM')
        end = time.monotonic()+a.seconds
        while time.monotonic() < end:
            t.send('DRIVE', a.throttle, a.steering)
            time.sleep(.05)
    finally:
        t.close()


if __name__ == '__main__':
    main()
