#!/usr/bin/env python3
"""Fase 1, E1 (frontal): ./run.sh run sumo robot0:=exp_push_front.py robot1:=none. Veja exp_push.py."""
from exp_push import Push


class PushFront(Push):
    mode = "front"


if __name__ == "__main__":
    PushFront().run()
