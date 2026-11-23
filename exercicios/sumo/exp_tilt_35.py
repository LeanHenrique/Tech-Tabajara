#!/usr/bin/env python3
"""Fase 1, E2: atacante inclinado em 35 graus. ./run.sh run sumo robot0:=exp_tilt_35.py robot1:=exp_facer.py"""
from exp_tilt import Tilt


class Tilt35(Tilt):
    phi_deg = 35


if __name__ == "__main__":
    Tilt35().run()
