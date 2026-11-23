#!/usr/bin/env python3
"""Observador do futebol: imprime a cada PERIODO s o placar, a bola e a pose dos 6 robôs. FERRAMENTA LOCAL.

Uso (dentro do container ROS, com um jogo de futebol rodando):
    python3 /ws/psr_exercicios/tools/watch_soccer.py [segundos_totais] [periodo]
"""
import json
import sys
import time

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from psr_hero.utils import pose_from_odom


def main():
    total = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    periodo = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
    rospy.init_node("watch_soccer", anonymous=True)
    poses, ball, score = {}, [None], [None]

    def on_odom(msg, i):
        poses[i] = pose_from_odom(msg)

    def on_ball(msg):
        p = msg.pose.pose.position
        ball[0] = (p.x, p.y, msg.twist.twist.linear.x, msg.twist.twist.linear.y)

    def on_state(msg):
        try:
            g = json.loads(msg.data)
            score[0] = (g["blue"], g["red"], g.get("time"))
        except (ValueError, KeyError):
            pass

    for i in range(6):
        rospy.Subscriber("/hero_%d/odom" % i, Odometry, on_odom, callback_args=i)
    rospy.Subscriber("/ball/odom", Odometry, on_ball)
    rospy.Subscriber("/game/state", String, on_state)
    t0 = time.time()
    while time.time() - t0 < total and not rospy.is_shutdown():
        time.sleep(periodo)
        if ball[0] is None or len(poses) < 6:
            continue
        b = ball[0]
        linha = "t=%5.1f placar=%s bola=(%+.2f,%+.2f v=%.2f)" % (time.time() - t0, score[0], b[0], b[1], (b[2] ** 2 + b[3] ** 2) ** 0.5)
        for i in range(6):
            x, y, yaw = poses[i]
            linha += " | h%d(%+.2f,%+.2f,%+4.0f)" % (i, x, y, yaw * 57.2958)
        print(linha, flush=True)


if __name__ == "__main__":
    main()
