#!/usr/bin/env python3
"""Sumô (RTech Tabajara), um processo por robô. Parâmetros: ~robot, ~opponent.

FASE 0: o nó sobe, assina as odometrias e publica (0, 0) desde o primeiro ciclo. A estratégia
entra nas fases seguintes (veja relatorio/DECISOES.md).

Regras do arquivo (formato do torneio): autossuficiente, só biblioteca padrão, numpy, rospy,
mensagens do ROS e psr_hero.utils; nenhuma leitura/escrita de arquivo; mesmo código nos dois lados.
"""
import math  # noqa: F401

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from psr_hero.utils import V_MAX, W_MAX, clamp, pose_from_odom, wrap_angle  # noqa: F401

RING_RADIUS = 0.40  # m, o robô perde quando o CENTRO sai do ringue
RATE_HZ = 20.0  # frequência do laço de controle


class Sumo:
    def __init__(self):
        rospy.init_node("sumo")
        self.robot = rospy.get_param("~robot", 0)
        self.opponent = rospy.get_param("~opponent", 1)
        self.me = None  # (x, y, yaw)
        self.speed = 0.0  # velocidade linear própria, m/s
        self.opp = None  # (x, y, yaw)

        self.pub = rospy.Publisher("/hero_%d/cmd_vel" % self.robot, Twist, queue_size=1)
        rospy.Subscriber("/hero_%d/odom" % self.robot, Odometry, self.on_me, queue_size=1)
        rospy.Subscriber("/hero_%d/odom" % self.opponent, Odometry, self.on_opp, queue_size=1)

    def on_me(self, msg):
        self.me = pose_from_odom(msg)
        self.speed = msg.twist.twist.linear.x

    def on_opp(self, msg):
        self.opp = pose_from_odom(msg)

    def compute_command(self, me, opp, speed, now):
        """Devolve (linear_x, angular_z). Fase 0: parado."""
        return 0.0, 0.0

    def publish(self, v, w):
        cmd = Twist()
        cmd.linear.x = clamp(v, -V_MAX, V_MAX)
        cmd.angular.z = clamp(w, -W_MAX, W_MAX)
        self.pub.publish(cmd)

    def step(self):
        """Um ciclo: SEMPRE publica algo. Sem dados ou com exceção, publica (0, 0)."""
        v, w = 0.0, 0.0
        if self.me is not None and self.opp is not None:
            try:
                v, w = self.compute_command(self.me, self.opp, self.speed, rospy.get_time())
            except Exception as exc:  # o nó nunca morre: um erro vira (0, 0)
                rospy.logwarn_throttle(2.0, "compute_command falhou: %r" % (exc,))
                v, w = 0.0, 0.0
        self.publish(v, w)

    def run(self):
        rate = rospy.Rate(RATE_HZ)
        self.publish(0.0, 0.0)  # a partida só começa quando os dois times publicam
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()


if __name__ == "__main__":
    Sumo().run()
