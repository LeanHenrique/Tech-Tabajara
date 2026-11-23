#!/usr/bin/env python3
"""Fase 1, E1: o hero_0 empurra o hero_1 PARADO (robot1:=none), de frente ou de lado.

  ./run.sh run sumo robot0:=exp_push_front.py robot1:=none
  ./run.sh run sumo robot0:=exp_push_side.py  robot1:=none

Valida H3 (resistência lateral fraca) e mede a velocidade do par em contato. Cada rodada: o hero_0 vai
até a posição de ataque e empurra a V_MAX até alguém cair; ao fim de N_ROUNDS rodadas, termina.
O hero_1 sem nó fica parado (o simulador zera o comando após 0,5 s) e ainda assim resiste.

Este módulo traz a lógica; exp_push_front.py e exp_push_side.py só escolhem o modo.

Modo side: o ponto de preparação fica ao lado do adversário, do lado do CENTRO do ringue, e o empurrão
vai para FORA (+y se o adversário está em y > 0; -y caso contrário), ao longo do eixo y do mundo.
"""
import math

import rospy  # noqa: F401

from exp_common import ExpNode, aim
from psr_hero.utils import V_MAX, W_MAX, clamp, wrap_angle

N_ROUNDS = 6
FLANK_OFFSET = 0.20  # m, distância lateral do ponto de preparação ao adversário (modo side)
CONTACT_DIST = 0.085  # m, abaixo disso conta como contato (raio 2 x 0,037 = 0,074)


class Push(ExpNode):
    name = "exp_push"
    mode = "front"

    def csv_name(self):
        return "exp_push_%s" % self.mode

    def __init__(self):
        ExpNode.__init__(self)
        self.phase = "approach"
        self.round0 = None
        self.cur_rnd = None

    def step(self):
        if self.round0 is None:
            self.round0 = self.cur_rnd = self.rnd
        if self.rnd != self.cur_rnd:  # a rodada mudou: o simulador teleportou os robôs
            self.cur_rnd, self.phase = self.rnd, "approach"
        if self.rnd - self.round0 >= N_ROUNDS:
            self.finish()
            return 0.0, 0.0
        me, opp = self.pose[0], self.pose[1]
        dist = math.hypot(opp[0] - me[0], opp[1] - me[1])

        if self.mode == "front":
            v, w, _, _ = aim(me, (opp[0], opp[1]))
            self.tag = "contact" if dist < CONTACT_DIST else "approach"
            return v, w

        out = 1.0 if opp[1] > 0.0 else -1.0  # sentido do empurrão (para fora, ao longo de y)
        heading = out * math.pi / 2.0
        if self.phase == "approach":
            wp = (opp[0], opp[1] - out * FLANK_OFFSET)  # do lado do centro do ringue
            v, w, d, _ = aim(me, wp, v_max=V_MAX * 0.9, stop_dist=0.03)
            self.tag = "approach"
            if d < 0.04:
                self.phase = "turn"
            return v, w
        err = wrap_angle(heading - me[2])
        if self.phase == "turn":
            self.tag = "turn"
            if abs(err) < 0.08:
                self.phase = "push"
            return 0.0, clamp(6.0 * err, -W_MAX, W_MAX)
        self.tag = "contact" if dist < CONTACT_DIST else "push"
        return V_MAX, clamp(6.0 * err, -W_MAX, W_MAX)
