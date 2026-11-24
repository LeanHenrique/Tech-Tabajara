#!/usr/bin/env python3
"""Observador do rouba-bandeira: imprime placar, poses, bandeiras, held e carrying a cada PERIODO s. FERRAMENTA LOCAL.

Uso (dentro do container ROS, com um jogo de CTF rodando):
    python3 /ws/psr_exercicios/tools/watch_ctf.py [segundos_totais] [periodo]
"""
import json
import sys
import time

import rospy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from psr_hero.utils import pose_from_odom


def main():
    total = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    periodo = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    rospy.init_node("watch_ctf", anonymous=True)
    poses, flags, state = {}, {}, [None]

    def on_odom(msg, i):
        poses[i] = pose_from_odom(msg)

    def on_flag(msg, cor):
        flags[cor] = (msg.point.x, msg.point.y)

    def on_state(msg):
        try:
            state[0] = json.loads(msg.data)
        except ValueError:
            pass

    for i in range(4):
        rospy.Subscriber("/hero_%d/odom" % i, Odometry, on_odom, callback_args=i)
    for cor in ("blue", "red"):
        rospy.Subscriber("/flag/%s" % cor, PointStamped, on_flag, callback_args=cor)
    rospy.Subscriber("/game/state", String, on_state)
    t0 = time.time()
    while time.time() - t0 < total and not rospy.is_shutdown():
        time.sleep(periodo)
        g = state[0]
        if g is None or len(poses) < 4 or len(flags) < 2:
            continue
        linha = "t=%5.1f rt=%5.1f placar=%s held=%s carry=%s bandeiras: az(%+.2f,%+.2f) vm(%+.2f,%+.2f)" % (
            time.time() - t0, g.get("round_time", -1), g.get("score"),
            "".join("H" if h else "." for h in g.get("held", [])),
            "".join("C" if c else "." for c in g.get("carrying", [])),
            flags["blue"][0], flags["blue"][1], flags["red"][0], flags["red"][1])
        for i in range(4):
            x, y, yaw = poses[i]
            linha += " | h%d(%+.2f,%+.2f,%+4.0f)" % (i, x, y, yaw * 57.2958)
        print(linha, flush=True)


if __name__ == "__main__":
    main()
