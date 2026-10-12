"""Controle Ackermann de teste; throttle e steering normalizados, sem ROS."""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class Target:
    box: tuple
    confidence: float

    @property
    def cx(self):
        return (self.box[0] + self.box[2]) / 2


def depth_for_cone(depth_m, box):
    """ROI central inferior: rejeita buracos e mistura ampla de profundidades.

    Heuristica de teste, nao segmentacao: deve ser conferida com trena.
    Nao preenche buracos nem reutiliza profundidade de frames anteriores.
    """
    x1, y1, x2, y2 = box
    h, w = depth_m.shape
    bw, bh = x2 - x1, y2 - y1
    left, right = max(0, int(x1 + .35*bw)), min(w, int(x1 + .65*bw))
    top, bottom = max(0, int(y1 + .45*bh)), min(h, int(y1 + .80*bh))
    if right - left < 4 or bottom - top < 4:
        return None, None
    roi = depth_m[top:bottom, left:right]
    values = roi[np.isfinite(roi) & (roi >= .10) & (roi <= 5)]
    if values.size < 16 or values.size / roi.size < .6:
        return None, (left, top, right, bottom)
    p20, p80 = np.percentile(values, [20, 80])
    if p80 - p20 > max(.06, .08*p20):
        return None, (left, top, right, bottom)
    return float(p20), (left, top, right, bottom)


class TargetLock:
    """Exige tres observacoes; nao troca de alvo durante uma aproximacao."""
    def __init__(self):
        self.previous = None
        self.count = 0

    def update(self, candidates, width):
        if self.previous is None:
            # Inicialmente exige um unico cone, evitando escolha arbitraria.
            matches = candidates if len(candidates) == 1 else []
        else:
            old = self.previous.box
            matches = []
            old_area = (old[2]-old[0])*(old[3]-old[1])
            for t in candidates:
                b = t.box
                area = (b[2]-b[0])*(b[3]-b[1])
                overlap = max(0, min(old[2], b[2])-max(old[0], b[0])) * max(0, min(old[3], b[3])-max(old[1], b[1]))
                iou = overlap / max(1, area + old_area - overlap)
                if abs(t.cx-self.previous.cx)/width < .12 and iou > .25:
                    matches.append(t)
        if len(matches) != 1:
            self.previous = None
            self.count = 0
            return None
        self.previous = matches[0]
        self.count += 1
        return self.previous if self.count >= 3 else None


def command(config, target, distance_m, width, frame_age):
    """Retorna estado, PWM 0..1, direcao -1..1. Zero e imediato."""
    if not math.isfinite(frame_age) or not 0 <= frame_age <= config['max_frame_age_s']:
        return 'FRAME_ATRASADO', 0., 0.
    if target is None:
        return 'SEM_ALVO_CONFIRMADO', 0., 0.
    if distance_m is None or not math.isfinite(distance_m) or not .10 <= distance_m <= 5:
        return 'PROFUNDIDADE_INVALIDA', 0., 0.
    stop = config['camera_recess_m'] + config['front_stop_m'] + config['braking_margin_m']
    if distance_m <= stop:
        return 'CONCLUIDO', 0., 0.
    error = (target.cx-width/2)/(width/2)
    # Servo nao permite girar parado. Alvo fora do alcance exige reposicionamento.
    if abs(error) > .75:
        return 'ALVO_FORA_DO_ALCANCE', 0., 0.
    steering = max(-1., min(1., error*config['steering_gain']*config['steering_sign']))
    if abs(error) < .04:
        steering = 0.
    fraction = min(1., (distance_m-stop)/(config['slowdown_distance_m']-stop))
    throttle = config['max_throttle']*fraction*(1-.5*abs(error))
    return 'APROXIMANDO', throttle, steering


def validate_config(c):
    numeric = ('confidence', 'camera_recess_m', 'front_stop_m', 'braking_margin_m',
               'slowdown_distance_m', 'max_throttle', 'steering_gain', 'max_frame_age_s')
    for key in numeric:
        if type(c[key]) not in (int, float) or not math.isfinite(c[key]) or c[key] < 0:
            raise ValueError(f'Configuracao invalida: {key}')
    if not 0 < c['confidence'] <= 1 or not 0 < c['max_throttle'] <= .25:
        raise ValueError('confidence 0..1; max_throttle deve estar em 0..0.25')
    if not 0 < c['max_frame_age_s'] <= .25 or not 0 < c['steering_gain'] <= 2:
        raise ValueError('max_frame_age_s <= 0.25; steering_gain em 0..2')
    if c['steering_sign'] not in (-1, 1) or type(c['hardware_calibrated']) is not bool:
        raise ValueError('steering_sign +/-1 e hardware_calibrated booleano')
    if type(c['cone_class']) is not int or c['cone_class'] < 0:
        raise ValueError('cone_class deve ser um inteiro nao negativo')
    stop = c['camera_recess_m'] + c['front_stop_m'] + c['braking_margin_m']
    if stop < .10 or c['front_stop_m'] <= 0 or c['slowdown_distance_m'] <= stop:
        raise ValueError('Distancias inconsistentes')
