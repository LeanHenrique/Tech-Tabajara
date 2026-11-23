#!/usr/bin/env python3
"""Fase 1, E2: atacante com o heading INCLINADO em PHI graus em relação à linha até o adversário.

Roda como hero_0 contra o exp_facer.py (hero_1). Valida H1 (empurrão frontal empata) e H2 (inclinar perto
dos eixos dá força lateral sem perder a frontal). Por rodada, o lado da inclinação é escolhido para que a
componente lateral leve o adversário para LONGE do centro do ringue; com PHI = 0 é o empurrão frontal puro.

Cada exp_tilt_<phi>.py só define PHI. N_ROUNDS rodadas; depois imprime EXPERIMENT_DONE.
"""
import math
import random

from exp_common import ExpNode
from psr_hero.utils import V_MAX, W_MAX, clamp, wrap_angle

N_ROUNDS = 10
CONTACT_DIST = 0.085


class Tilt(ExpNode):
    name = "exp_tilt"
    phi_deg = 0

    def csv_name(self):
        return "exp_tilt_%02d" % self.phi_deg

    def __init__(self):
        ExpNode.__init__(self)
        self.round0 = self.cur_rnd = None
        self.s = None
        self.rng = random.Random()

    def step(self):
        if self.round0 is None:
            self.round0 = self.cur_rnd = self.rnd
        if self.rnd != self.cur_rnd:  # nova rodada: o simulador teleportou os robôs
            self.cur_rnd, self.s = self.rnd, None
        if self.rnd - self.round0 >= N_ROUNDS:
            self.finish()
            return 0.0, 0.0
        me, opp = self.pose[0], self.pose[1]
        ux, uy = opp[0] - me[0], opp[1] - me[1]
        dist = math.hypot(ux, uy)
        if self.s is None:
            # n = (-uy, ux): a componente lateral s*n leva o adversário para fora do centro se s*(n . opp) > 0
            cross = -uy * opp[0] + ux * opp[1]
            self.s = 1.0 if cross > 1e-6 else -1.0 if cross < -1e-6 else self.rng.choice((-1.0, 1.0))
        heading = math.atan2(uy, ux) + self.s * math.radians(self.phi_deg)
        err = wrap_angle(heading - me[2])
        self.tag = ("contact" if dist < CONTACT_DIST else "approach") + ("_s%+d" % int(self.s))
        return V_MAX * max(0.0, math.cos(err)), clamp(5.0 * err, -W_MAX, W_MAX)
