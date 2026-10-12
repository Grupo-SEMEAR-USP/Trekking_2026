"""Distancia monocular aproximada: K = altura da caixa em pixels * distancia."""
from collections import deque
import math
import numpy as np


def full_box_height(box, shape):
    h, w = shape[:2]
    x1, y1, x2, y2 = box
    if not all(math.isfinite(v) for v in box):
        return None
    # Cone cortado pelo enquadramento nao serve para estimar distancia.
    if min(x1, y1) <= 3 or x2 >= w-3 or y2 >= h-3 or x2-x1 < 8 or y2-y1 < 20:
        return None
    return y2-y1


class CalibrationSamples:
    def __init__(self):
        self.samples = deque(maxlen=30)

    def update(self, target, shape, age):
        height = full_box_height(target.box, shape) if target and 0 <= age <= .25 else None
        if height is None:
            self.samples.clear()
        else:
            self.samples.append(height)

    def calibration(self, distance, shape, source, backend, model_hash):
        if not math.isfinite(distance) or not .3 <= distance <= 2:
            raise ValueError('Distancia de referencia deve estar entre 0.30 e 2 m')
        if len(self.samples) < 20:
            raise ValueError('Espere ao menos 20 deteccoes estaveis do cone inteiro')
        p10, median, p90 = np.percentile(self.samples, [10, 50, 90])
        if (p90-p10)/median > .1:
            raise ValueError('Caixa instavel; mantenha camera e cone parados')
        return {'version': 1, 'pixel_meter': float(median*distance),
                'reference_distance_m': distance, 'reference_height_px': float(median),
                'image_size': [shape[1], shape[0]], 'source': source, 'backend': backend,
                'model_sha256': model_hash}


def estimate_distance(calibration, target, shape, source, backend, model_hash):
    if not calibration or target is None:
        return None
    if (calibration.get('version') != 1 or calibration.get('image_size') != [shape[1],shape[0]] or
            calibration.get('source') != source or calibration.get('backend') != backend or
            calibration.get('model_sha256') != model_hash):
        return None
    height = full_box_height(target.box, shape)
    factor = calibration.get('pixel_meter')
    if height is None or type(factor) not in (float,int) or not math.isfinite(factor) or factor <= 0:
        return None
    distance = factor/height
    return distance if .10 <= distance <= 5 else None
