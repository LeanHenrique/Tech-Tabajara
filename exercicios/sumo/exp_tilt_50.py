#!/usr/bin/env python3
"""Fase 1, E2: atacante inclinado em 50 graus. ./run.sh run sumo robot0:=exp_tilt_50.py robot1:=exp_facer.py"""
from exp_tilt import Tilt


class Tilt50(Tilt):
    phi_deg = 50


if __name__ == "__main__":
    Tilt50().run()
