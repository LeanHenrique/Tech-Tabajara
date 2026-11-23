#!/usr/bin/env python3
"""Fase 1, E2: atacante inclinado em 0 graus. ./run.sh run sumo robot0:=exp_tilt_00.py robot1:=exp_facer.py"""
from exp_tilt import Tilt


class Tilt00(Tilt):
    phi_deg = 0


if __name__ == "__main__":
    Tilt00().run()
