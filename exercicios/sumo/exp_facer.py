#!/usr/bin/env python3
"""Fase 1, E2: adversário "facer": encara o outro robô o tempo todo e empurra de frente à velocidade máxima.

Simula um adversário bom (provavelmente o Tech). Usa o robô definido por ~robot (hero_1 nos experimentos).
Não grava CSV (quem grava é o atacante inclinado).

  ./run.sh run sumo robot0:=exp_tilt_35.py robot1:=exp_facer.py
"""
from exp_common import ExpNode, aim


class Facer(ExpNode):
    name = "exp_facer"

    def csv_name(self):
        return None

    def step(self):
        me, opp = self.pose[self.me], self.pose[self.opp]
        v, w, _, _ = aim(me, (opp[0], opp[1]))
        self.tag = "facer"
        return v, w


if __name__ == "__main__":
    Facer().run()
