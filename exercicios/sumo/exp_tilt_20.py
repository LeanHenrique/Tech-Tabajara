#!/usr/bin/env python3
"""Fase 1, E2: atacante inclinado em 20 graus. ./run.sh run sumo robot0:=exp_tilt_20.py robot1:=exp_facer.py"""
from exp_tilt import Tilt


class Tilt20(Tilt):
    phi_deg = 20


if __name__ == "__main__":
    Tilt20().run()
