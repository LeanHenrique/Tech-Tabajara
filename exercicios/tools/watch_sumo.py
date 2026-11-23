#!/usr/bin/env python3
"""Observador do sumô: imprime a cada PERIODO s o round_time, o placar e a pose dos dois robôs. FERRAMENTA LOCAL.

Uso (dentro do container ROS, com um jogo de sumô rodando):
    python3 /ws/psr_exercicios/tools/watch_sumo.py [segundos_totais] [periodo]
"""
import json
import math
import sys
import time

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from psr_hero.utils import pose_from_odom


def main():
    total = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    periodo = float(sys.argv[2]) if len(sys.argv) > 2 else 0.4
    rospy.init_node("watch_sumo", anonymous=True)
    poses, state = {}, [None]

    def on_odom(msg, i):
        poses[i] = pose_from_odom(msg)

    def on_state(msg):
        try:
            state[0] = json.loads(msg.data)
        except ValueError:
            pass

    for i in (0, 1):
        rospy.Subscriber("/hero_%d/odom" % i, Odometry, on_odom, callback_args=i)
    rospy.Subscriber("/game/state", String, on_state)
    t0 = time.time()
    while time.time() - t0 < total and not rospy.is_shutdown():
        time.sleep(periodo)
        if len(poses) < 2 or state[0] is None:
            continue
        g = state[0]
        a, b = poses[0], poses[1]
        print("rt=%5.1f wins=%s | h0(%+.2f,%+.2f,%+4.0f r=%.2f) | h1(%+.2f,%+.2f,%+4.0f r=%.2f) | dist=%.2f" % (
            g.get("round_time", -1), g.get("wins"),
            a[0], a[1], math.degrees(a[2]), math.hypot(a[0], a[1]),
            b[0], b[1], math.degrees(b[2]), math.hypot(b[0], b[1]),
            math.hypot(a[0] - b[0], a[1] - b[1])), flush=True)


if __name__ == "__main__":
    main()
