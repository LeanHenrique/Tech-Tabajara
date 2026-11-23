#!/usr/bin/env python3
"""Fase 1, E1 (lateral): ./run.sh run sumo robot0:=exp_push_side.py robot1:=none. Veja exp_push.py."""
from exp_push import Push


class PushSide(Push):
    mode = "side"


if __name__ == "__main__":
    PushSide().run()
