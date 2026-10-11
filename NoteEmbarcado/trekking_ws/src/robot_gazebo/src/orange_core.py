"""Visão e missão testáveis sem ROS, câmera ou motores. Unidades em SI."""
from dataclasses import dataclass, replace
import math
from statistics import median

ORANGE_NAMES = frozenset(('orange_cone', 'large_orange_cone'))


def orange_class_ids(names):
    items = names.items() if isinstance(names, dict) else enumerate(names)
    ids = sorted(int(k) for k, name in items if str(name).lower() in ORANGE_NAMES)
    if not ids:
        raise ValueError('O modelo não contém orange_cone/large_orange_cone. Use o modelo de cones fornecido.')
    return ids


@dataclass(frozen=True)
class Detection:
    track_id: int
    x: float
    y: float
    width: float
    height: float
    confidence: float
    class_id: int
    depth_m: float = math.nan
    bearing_rad: float = math.nan  # Esquerda positiva, a partir dos intrínsecos RGB.
    half_width_rad: float = math.nan
    range_source: str = 'camera'
    range_stamp: float = 0.


def box_detection(xyxy, confidence, class_id, image_width, image_height):
    x1, y1, x2, y2 = map(float, xyxy)
    if not all(math.isfinite(v) for v in (x1, y1, x2, y2, confidence)) or not 0 <= confidence <= 1:
        return None
    x1, x2 = max(0., x1), min(float(image_width), x2)
    y1, y2 = max(0., y1), min(float(image_height), y2)
    if x2-x1 < 3 or y2-y1 < 5:
        return None
    return Detection(-1, min(image_width-1., (x1+x2)/2), min(image_height-1., (y1+y2)/2),
                     x2-x1, y2-y1, float(confidence), int(class_id))


def cone_depth_m(depth, detection, scale, minimum=0.10, maximum=10., minimum_samples=8):
    """Mediana no interior da caixa, evitando bordas e buracos da profundidade.

    Retorna NaN se houver poucos pontos ou dispersão grande. Não preenche buracos
    com a leitura antiga: sem profundidade atual, a missão para.
    """
    height, width = len(depth), len(depth[0]) if len(depth) else 0
    if not width or not math.isfinite(scale) or scale <= 0:
        return math.nan
    rx, ry = max(2, int(detection.width*.15)), max(2, int(detection.height*.20))
    cx, cy = int(round(detection.x)), int(round(detection.y))
    values = [float(depth[y][x])*scale
              for y in range(max(0, cy-ry), min(height, cy+ry+1))
              for x in range(max(0, cx-rx), min(width, cx+rx+1))
              if math.isfinite(float(depth[y][x])) and minimum <= float(depth[y][x])*scale <= maximum]
    if len(values) < minimum_samples:
        return math.nan
    values.sort()
    middle = median(values)
    # Um fundo dominante ou uma mistura muito dispersa não autoriza aproximação.
    if values[int(.75*(len(values)-1))] - values[int(.25*(len(values)-1))] > max(.08, middle*.15):
        return math.nan
    return middle


class OrangeTracker:
    """Associação local por posição/tamanho; IDs nunca são reciclados no processo.

    Não é reconhecimento persistente de cones depois de oclusões prolongadas.
    """
    def __init__(self, gate_px=90., retention_s=1.5):
        self.gate, self.retention = gate_px, retention_s
        self.tracks = {}
        self.next_id = 1

    def update(self, detections, now):
        self.tracks = {i: (d, t) for i, (d, t) in self.tracks.items() if 0 <= now-t <= self.retention}
        pairs = []
        for j, d in enumerate(detections):
            for i, (old, _) in self.tracks.items():
                distance = math.hypot(d.x-old.x, d.y-old.y)
                ratio = max(d.width/old.width, old.width/d.width, d.height/old.height, old.height/d.height)
                if distance <= self.gate and ratio <= 2.5:
                    pairs.append((distance+10.*abs(math.log(ratio)), i, j))
        assigned, used = {}, set()
        for _, i, j in sorted(pairs):
            if j not in assigned and i not in used:
                assigned[j] = i
                used.add(i)
        result = []
        for j, d in enumerate(detections):
            i = assigned.get(j)
            if i is None:
                i, self.next_id = self.next_id, self.next_id+1
            tracked = replace(d, track_id=i)
            self.tracks[i] = tracked, now
            result.append(tracked)
        return result


@dataclass(frozen=True)
class MissionConfig:
    mode: str = 'preview'  # preview, align, mission
    image_width: int = 640
    image_height: int = 480
    max_cones: int = 4
    wheelbase_m: float = .30
    max_steering_rad: float = .35
    steering_gain_rad: float = .8
    forward_speed_m_s: float = .10
    minimum_speed_m_s: float = .025
    reverse_speed_m_s: float = .08
    distance_gain: float = .6
    camera_to_front_m: float = .08  # Confirmado: câmera 8 cm atrás da frente.
    arrival_gap_m: float = .20       # Confirmado: frente do robô a 20 cm do cone.
    arrival_tolerance_m: float = .015
    minimum_gap_m: float = .10
    retreat_gap_m: float = .90
    next_cone_min_gap_m: float = 1.10
    tolerance_px: float = 12.
    resume_tolerance_px: float = 20.
    confirmation_frames: int = 3
    detection_timeout_s: float = .4
    scan_timeout_s: float = .35
    target_lost_timeout_s: float = 2.
    acquire_timeout_s: float = 30.
    approach_timeout_s: float = 120.
    retreat_timeout_s: float = 20.
    search_timeout_s: float = 17.
    blocked_timeout_s: float = 8.
    blink_count: int = 3
    blink_period_s: float = .4
    # Dimensões e posição LiDAR provisórias; referência base = eixo traseiro.
    robot_front_m: float = .22
    robot_rear_m: float = .10
    robot_half_width_m: float = .12
    safety_margin_m: float = .04
    reaction_time_s: float = .3
    braking_deceleration_m_s2: float = .25
    lidar_x_m: float = .0
    lidar_y_m: float = .0
    lidar_yaw_rad: float = .0
    camera_lateral_m: float = .0
    lidar_range_tolerance_m: float = .25
    lidar_zero_is_clear: bool = True  # Driver Slamtec representa sem retorno como +inf.

    def __post_init__(self):
        if self.mode not in ('preview', 'align', 'mission'):
            raise ValueError('mode: use preview, align ou mission')
        for name in ('image_width', 'image_height', 'max_cones', 'confirmation_frames', 'blink_count'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ValueError(f'{name} deve ser inteiro positivo')
        for name in self.__dataclass_fields__:
            value = getattr(self, name)
            if isinstance(value, float) and not math.isfinite(value):
                raise ValueError(f'{name} não finito')
            if isinstance(value, float) and name not in ('camera_to_front_m','lidar_x_m','lidar_y_m','lidar_yaw_rad','camera_lateral_m') and value <= 0:
                raise ValueError(f'{name} deve ser positivo')
        if self.camera_to_front_m < 0 or self.image_width < 3 or self.image_height < 3:
            raise ValueError('Câmera/geometria inválida')
        if not 0 < self.max_steering_rad < math.pi/2 or self.minimum_speed_m_s > self.forward_speed_m_s:
            raise ValueError('Limites de direção/velocidade inválidos')
        if not self.minimum_gap_m < self.arrival_gap_m < self.retreat_gap_m <= self.next_cone_min_gap_m:
            raise ValueError('Distâncias devem satisfazer mínimo < chegada < recuo <= próximo cone')
        if not self.tolerance_px < self.resume_tolerance_px < (self.image_width-1)/2:
            raise ValueError('Tolerâncias de pixel inválidas')


@dataclass(frozen=True)
class Output:
    v: float = 0.
    omega: float = 0.
    led: bool = False
    state: str = 'IDLE'
    reason: str = ''
    visited: int = 0
    target: object = None


class LaserGuard:
    """Verifica cobertura e retângulo varrido na frente ou atrás. Não desvia obstáculos."""
    def __init__(self, config):
        self.c = config
        self.points, self.valid_angles = [], []
        self.stamp = self.received = None

    def update(self, ranges, angle_min, increment, range_min, range_max, stamp, ros_now, now):
        if (not all(math.isfinite(v) for v in (angle_min, increment, range_min, range_max, stamp, ros_now, now))
                or abs(increment) < 1e-6 or not 0 <= range_min < range_max or len(ranges) < 20
                or stamp <= 0 or not -.05 <= ros_now-stamp <= self.c.scan_timeout_s):
            self.received = None
            return False
        if self.stamp is not None and stamp <= self.stamp:
            return False
        self.points, self.valid_angles = [], []
        for index, distance in enumerate(ranges):
            angle = angle_min + index*increment + self.c.lidar_yaw_rad
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if math.isfinite(distance) and range_min <= distance <= range_max:
                self.points.append((self.c.lidar_x_m+distance*math.cos(angle),
                                    self.c.lidar_y_m+distance*math.sin(angle)))
                self.valid_angles.append(angle)
            elif distance == math.inf and self.c.lidar_zero_is_clear:
                self.valid_angles.append(angle)
        self.stamp, self.received = stamp, now
        return True

    def fresh(self, now, ros_now):
        return (self.received is not None and 0 <= now-self.received <= self.c.scan_timeout_s
                and -.05 <= ros_now-self.stamp <= self.c.scan_timeout_s)

    def permits(self, v, omega, now, ros_now):
        if not self.fresh(now, ros_now):
            return False, 'lidar_sem_amostra_recente'
        if len(self.points) < 3:
            return False, 'lidar_sem_retornos_suficientes'
        # Exige cobertura nos 160 graus ao redor do sentido de deslocamento.
        heading = 0. if v >= 0 else math.pi
        bins = set()
        for angle in self.valid_angles:
            relative = math.atan2(math.sin(angle-heading), math.cos(angle-heading))
            if -math.radians(80) <= relative < math.radians(80):
                bins.add(int((relative+math.radians(80))/math.radians(10)))
        if len(bins) < 16:
            return False, 'lidar_cobertura_insuficiente'
        c = self.c
        # Distância até parar com reação + frenagem; não representa uma calibração dos freios.
        duration = c.reaction_time_s + abs(v)/(2*c.braking_deceleration_m_s2)
        for index in range(13):
            t = duration*index/12
            yaw = omega*t
            if abs(omega) < 1e-6:
                cx, cy = v*t, 0.
            else:
                cx, cy = v/omega*math.sin(yaw), v/omega*(1-math.cos(yaw))
            co, si = math.cos(yaw), math.sin(yaw)
            for px, py in self.points:
                x, y = co*(px-cx)+si*(py-cy), -si*(px-cx)+co*(py-cy)
                if (-c.robot_rear_m-c.safety_margin_m <= x <= c.robot_front_m+c.safety_margin_m
                        and abs(y) <= c.robot_half_width_m+c.safety_margin_m):
                    return False, 'obstaculo_na_trajetoria'
        return True, ''


class OrangeMission:
    def __init__(self, config):
        self.c = config
        self.guard = LaserGuard(config)
        self.state, self.reason = 'IDLE', ''
        self.visited_ids = set()
        self.target_id = None
        self.detections = []
        self.frame_stamp = self.frame_time = self.target_seen = None
        self.state_since = 0.
        self.center_frames = self.arrival_frames = 0
        self.pending_id, self.pending_frames = None, 0
        self.blocked_since = None
        self.arrival_range_stamp = None
        self.camera_missing_since = None
        self.target_started = 0.
        self.range_memory = {}

    def transition(self, state, now, reason=''):
        self.state, self.state_since, self.reason = state, now, reason
        self.center_frames = self.arrival_frames = 0
        self.pending_id, self.pending_frames = None, 0
        self.blocked_since = None
        self.arrival_range_stamp = None

    def start(self, now):
        self.visited_ids.clear()
        self.target_id = self.target_seen = None
        self.camera_missing_since = None
        self.range_memory.clear()
        # Não reutiliza uma imagem recebida antes do início/reinício.
        self.frame_time = None
        self.detections = []
        self.transition('ACQUIRE', now)

    def stop(self, now):
        self.transition('IDLE', now, 'parada_solicitada')

    def target(self):
        return next((d for d in self.detections if d.track_id == self.target_id), None)

    def gap(self, detection):
        return detection.depth_m-self.c.camera_to_front_m

    def valid_depth(self, d):
        return math.isfinite(d.depth_m) and .10 <= d.depth_m <= 10.

    def resolve_range(self,d,stamp,ros_now,now):
        """Associa o cone ao LiDAR por ângulo, com os deslocamentos medidos.

        A câmera fornece a classe e o centro. LiDAR complementa a distância se a
        profundidade tiver buracos; discordância grande entre sensores suspende o alvo.
        """
        camera_valid = self.valid_depth(d)
        memory = self.range_memory.get(d.track_id)
        # A profundidade inicial autentica a associação de alcance. Um fundo
        # atrás do cone não pode virar um cone só porque está no mesmo ângulo.
        if (not self.guard.fresh(now,ros_now) or not math.isfinite(d.bearing_rad)
                or not math.isfinite(d.half_width_rad) or d.half_width_rad <= 0):
            if camera_valid:
                self.range_memory[d.track_id] = d.depth_m,now
            return replace(d,range_stamp=stamp)
        camera_x = self.c.robot_front_m-self.c.camera_to_front_m
        axial = []
        tolerance = max(.012,d.half_width_rad*.65)
        for px,py in self.guard.points:
            x,y = px-camera_x,py-self.c.camera_lateral_m
            bearing = math.atan2(y,x)
            if x >= .10 and abs(math.atan2(math.sin(bearing-d.bearing_rad),math.cos(bearing-d.bearing_rad))) <= tolerance:
                axial.append(x)
        axial.sort()
        cluster = []
        for x in axial:
            group = [v for v in axial if x <= v <= x+.06]
            if len(group) >= 3:
                cluster = group
                break
        if not cluster:
            if camera_valid:
                self.range_memory[d.track_id] = d.depth_m,now
            return replace(d,range_stamp=stamp)
        distance = median(cluster)
        if camera_valid and abs(distance-d.depth_m) > self.c.lidar_range_tolerance_m:
            return replace(d,depth_m=math.nan,range_source='sensores_discordam',range_stamp=stamp)
        if not camera_valid:
            if memory is None or not 0 <= now-memory[1] <= 1.:
                return replace(d,range_stamp=stamp)
            max_change = .10+(self.c.forward_speed_m_s+self.c.reverse_speed_m_s)*(now-memory[1])
            if abs(distance-memory[0]) > max_change:
                return replace(d,range_stamp=stamp)
        self.range_memory[d.track_id] = distance,now
        return replace(d,depth_m=distance,range_source='lidar',range_stamp=self.guard.stamp)

    def observe(self, detections, stamp, ros_now, now):
        if (not all(math.isfinite(v) for v in (stamp, ros_now, now)) or stamp <= 0
                or not -.05 <= ros_now-stamp <= self.c.detection_timeout_s):
            self.frame_time = None
            self.center_frames = self.arrival_frames = 0
            return False
        if self.frame_stamp is not None and stamp <= self.frame_stamp:
            return False
        if self.frame_time is None or now-self.frame_time > self.c.detection_timeout_s:
            self.center_frames = self.arrival_frames = 0
            self.pending_id, self.pending_frames = None, 0
        self.frame_stamp, self.frame_time = stamp, now
        self.range_memory = {i: value for i,value in self.range_memory.items() if 0 <= now-value[1] <= 3.}
        self.detections = [self.resolve_range(d,stamp,ros_now,now) for d in detections if d.track_id > 0
                           and all(math.isfinite(v) for v in (d.x,d.y,d.width,d.height,d.confidence))
                           and 0 <= d.x < self.c.image_width and 0 <= d.y < self.c.image_height
                           and d.width > 0 and d.height > 0 and 0 <= d.confidence <= 1]
        if self.state in ('ACQUIRE', 'SEARCH_RIGHT'):
            candidates = [d for d in self.detections if d.track_id not in self.visited_ids
                          and (self.valid_depth(d) or self.c.mode == 'preview')]
            if self.state == 'SEARCH_RIGHT':
                candidates = [d for d in candidates if d.x >= (self.c.image_width-1)/2+self.c.resume_tolerance_px
                              and self.gap(d) >= self.c.next_cone_min_gap_m]
            candidate = min(candidates,
                            key=(lambda d: (d.depth_m,abs(d.x-(self.c.image_width-1)/2)))
                            if self.state == 'SEARCH_RIGHT' else (lambda d: abs(d.x-(self.c.image_width-1)/2)),
                            default=None)
            if candidate is not None:
                self.pending_frames = self.pending_frames+1 if candidate.track_id == self.pending_id else 1
                self.pending_id = candidate.track_id
            else:
                self.pending_id, self.pending_frames = None, 0
        d = self.target()
        if d is None or not self.valid_depth(d):
            self.center_frames = self.arrival_frames = 0
        else:
            self.target_seen = now
            centered = abs(d.x-(self.c.image_width-1)/2) <= self.c.tolerance_px
            self.center_frames = self.center_frames+1 if centered else 0
            arrived = centered and self.c.minimum_gap_m <= self.gap(d) <= self.c.arrival_gap_m+self.c.arrival_tolerance_m
            if not arrived:
                self.arrival_frames = 0
                self.arrival_range_stamp = None
            elif d.range_stamp != self.arrival_range_stamp:
                self.arrival_frames += 1
                self.arrival_range_stamp = d.range_stamp
        return True

    def output(self, v=0., omega=0., led=False, reason=None):
        return Output(v,omega,led,self.state,self.reason if reason is None else reason,len(self.visited_ids),self.target())

    def drive(self, v, steering, now, ros_now):
        omega = v*math.tan(steering)/self.c.wheelbase_m
        allowed, reason = self.guard.permits(v,omega,now,ros_now)
        if not allowed:
            if self.blocked_since is None:
                self.blocked_since = now
            if now-self.blocked_since > self.c.blocked_timeout_s:
                self.transition('FAULT',now,reason)
            return self.output(reason=reason)
        self.blocked_since = None
        return self.output(v,omega,reason='')

    def step(self, now, ros_now):
        c = self.c
        if self.state in ('IDLE','COMPLETE','FAULT'):
            return self.output()
        if self.state == 'BLINK':
            elapsed = now-self.state_since
            if elapsed < c.blink_count*c.blink_period_s:
                return self.output(led=(elapsed % c.blink_period_s) < c.blink_period_s/2)
            self.transition('COMPLETE' if len(self.visited_ids) >= c.max_cones else 'RETREAT',now)
            return self.output()
        fresh = (self.frame_time is not None and 0 <= now-self.frame_time <= c.detection_timeout_s
                 and -.05 <= ros_now-self.frame_stamp <= c.detection_timeout_s)
        if not fresh:
            if self.camera_missing_since is None:
                self.camera_missing_since = now
            if now-self.camera_missing_since > (c.acquire_timeout_s if self.state == 'ACQUIRE' else c.target_lost_timeout_s):
                self.transition('FAULT',now,'camera_sem_deteccao_recente')
            return self.output(reason='aguardando_camera')
        self.camera_missing_since = None
        if c.mode != 'preview' and not self.guard.fresh(now,ros_now):
            return self.drive(0.,0.,now,ros_now)
        if self.state in ('ACQUIRE','SEARCH_RIGHT'):
            if self.pending_frames >= c.confirmation_frames:
                self.target_id = self.pending_id
                self.target_seen = now
                self.target_started = now
                self.transition('ALIGN',now)
                return self.output()
            if self.state == 'ACQUIRE':
                if now-self.state_since > c.acquire_timeout_s:
                    self.transition('FAULT',now,'nenhum_cone_laranja_com_profundidade')
                return self.output()
            if now-self.state_since > c.search_timeout_s:
                self.transition('FAULT',now,'proximo_cone_nao_encontrado_a_direita')
                return self.output()
            if self.pending_id is not None:
                return self.output(reason='confirmando_proximo_cone')
            return self.drive(c.forward_speed_m_s,-c.max_steering_rad,now,ros_now)
        d = self.target()
        if c.mode == 'preview':
            return self.output(reason='visualizacao_sem_movimento')
        if d is None or not self.valid_depth(d):
            if self.target_seen is None or now-self.target_seen > c.target_lost_timeout_s:
                self.transition('FAULT',now,'alvo_perdido_ou_sem_profundidade')
            return self.output(reason='aguardando_alvo_atual')
        gap = self.gap(d)
        if gap < c.minimum_gap_m:
            self.transition('FAULT',now,'cone_proximo_demais')
            return self.output()
        if self.state == 'RETREAT':
            if gap >= c.retreat_gap_m:
                self.target_id = None
                self.transition('SEARCH_RIGHT',now)
                return self.output()
            if now-self.state_since > c.retreat_timeout_s:
                self.transition('FAULT',now,'recuo_sem_conclusao')
                return self.output()
            if abs(d.x-(c.image_width-1)/2) > c.resume_tolerance_px*3:
                self.transition('FAULT',now,'cone_saiu_do_eixo_durante_recuo')
                return self.output()
            return self.drive(-c.reverse_speed_m_s,0.,now,ros_now)
        if now-self.target_started > c.approach_timeout_s:
            self.transition('FAULT',now,'aproximacao_sem_conclusao')
            return self.output()
        error = d.x-(c.image_width-1)/2
        steering = max(-c.max_steering_rad,min(c.max_steering_rad,-c.steering_gain_rad*error/((c.image_width-1)/2)))
        if c.mode == 'preview':
            return self.output(reason='visualizacao_sem_movimento')
        if c.mode == 'align':
            if abs(error) <= c.tolerance_px:
                return self.output(reason='centralizado' if self.center_frames >= c.confirmation_frames else 'confirmando_centro')
            if gap <= c.arrival_gap_m+c.arrival_tolerance_m:
                return self.output(reason='cone_perto_demais_para_alinhar')
            return self.drive(c.forward_speed_m_s,steering,now,ros_now)
        if gap <= c.arrival_gap_m+c.arrival_tolerance_m:
            if self.arrival_frames >= c.confirmation_frames:
                self.visited_ids.add(self.target_id)
                self.transition('BLINK',now)
                return self.output(led=True)
            return self.output(reason='confirmando_chegada' if abs(error) <= c.tolerance_px else 'perto_sem_alinhamento')
        if self.state == 'ALIGN' and self.center_frames >= c.confirmation_frames:
            self.transition('APPROACH',now)
        elif self.state == 'APPROACH' and abs(error) > c.resume_tolerance_px:
            self.transition('ALIGN',now)
        if self.state == 'ALIGN' and abs(error) <= c.tolerance_px:
            return self.output(reason='confirmando_centro')
        speed = min(c.forward_speed_m_s,max(c.minimum_speed_m_s,c.distance_gain*(gap-c.arrival_gap_m)))
        return self.drive(speed,steering,now,ros_now)
