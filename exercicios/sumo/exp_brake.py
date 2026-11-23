#!/usr/bin/env python3
"""Fase 1, E3: distância e tempo de frenagem do robô (e resposta de aceleração).

Uso:  ./run.sh run sumo robot0:=exp_brake.py robot1:=none

O hero_0 anda em linha reta (yaw 0) por T_ACC s a uma velocidade V e comanda 0; mede-se até parar.
Alterna frente e ré para ficar dentro do ringue. Cada tentativa vira um trecho do CSV com tag
acc_<k>, brk_<k> (frenagem) e idle. Valida a margem de R_SAFE do sumô (briefing, H e Fase 1).
"""
import math

import rospy

from exp_common import ExpNode

SPEEDS = [0.10, 0.20]
REPS = 3  # por velocidade
T_ACC = 1.0  # s acelerando antes de comandar 0
T_BRK_MAX = 2.0  # s no máximo freando
T_IDLE = 0.6  # s parado entre tentativas
V_STOP = 0.004  # m/s: abaixo disso considera parado


class Brake(ExpNode):
    name = "exp_brake"

    def __init__(self):
        ExpNode.__init__(self)
        self.trials = []
        direction = 1.0
        for v in SPEEDS:
            for _ in range(REPS):
                self.trials.append(direction * v)
                direction = -direction
        self.k = 0
        self.phase = "wait_reset"
        self.t_phase = None
        self.last = None  # (t, x, y)
        self.speed = 0.0
        self.last_rt = None

    def est_speed(self, now):
        x, y = self.pose[0][0], self.pose[0][1]
        if self.last is not None and now > self.last[0]:
            self.speed = math.hypot(x - self.last[1], y - self.last[2]) / (now - self.last[0])
        self.last = (now, x, y)

    def step(self):
        now = rospy.get_time()
        self.est_speed(now)
        if self.phase == "wait_reset":  # espera o início de uma rodada (round_time voltando a ~0) para ter 40 s livres
            self.tag = "wait"
            if self.last_rt is not None and self.rt < self.last_rt - 1.0:
                self.phase, self.t_phase = "acc", now
            self.last_rt = self.rt
            return 0.0, 0.0
        if self.k >= len(self.trials):
            self.finish()
            return 0.0, 0.0
        v_cmd = self.trials[self.k]
        if self.phase == "acc":
            self.tag = "acc_%d_v%+.2f" % (self.k, v_cmd)
            if now - self.t_phase >= T_ACC:
                self.phase, self.t_phase = "brk", now
                self.tag = "brk_%d_v%+.2f" % (self.k, v_cmd)
            return v_cmd, 0.0
        if self.phase == "brk":
            self.tag = "brk_%d_v%+.2f" % (self.k, v_cmd)
            if (now - self.t_phase >= 0.2 and self.speed < V_STOP) or now - self.t_phase >= T_BRK_MAX:
                self.phase, self.t_phase = "idle", now
            return 0.0, 0.0
        self.tag = "idle"
        if now - self.t_phase >= T_IDLE:
            self.k += 1
            self.phase, self.t_phase = "acc", now
        return 0.0, 0.0


if __name__ == "__main__":
    Brake().run()
