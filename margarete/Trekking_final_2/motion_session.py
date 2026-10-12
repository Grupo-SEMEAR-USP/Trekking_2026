"""Pausa instantanea por falha breve do alvo; nunca continua sem observacao."""
class MotionSession:
    def __init__(self, recovery_s=.6):
        self.recovery_s=recovery_s
        self.phase='PARADO'
        self.pause_at=None
        self.reason=''

    @property
    def active(self): return self.phase=='SEGUINDO'

    @property
    def requested(self): return self.phase in ('SEGUINDO','PAUSA_ALVO')

    def start(self):
        self.phase='SEGUINDO'; self.pause_at=None; self.reason=''

    def stop(self):
        self.phase='PARADO'; self.pause_at=None; self.reason='PARADA_MANUAL'

    def update(self, state, now):
        if not self.requested: return None
        if self.phase=='PAUSA_ALVO':
            if now-self.pause_at >= self.recovery_s:
                self.phase='REINICIAR_ESPACO'; self.reason='ALVO_PERDIDO'
                return None
            if state=='APROXIMANDO':
                self.start()
                return 'RESUME'
            if state!='SEM_ALVO_CONFIRMADO':
                self.phase='REINICIAR_ESPACO'; self.reason=state
            return None
        if state=='APROXIMANDO': return 'DRIVE'
        self.reason=state
        if state=='SEM_ALVO_CONFIRMADO':
            self.phase='PAUSA_ALVO'; self.pause_at=now
        else:
            self.phase='REINICIAR_ESPACO'
        return 'STOP'
