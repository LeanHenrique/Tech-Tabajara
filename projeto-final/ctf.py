#!/usr/bin/env python3
"""Rouba-bandeira (RTech Tabajara): um processo por robô.

Parâmetros: ~robot (id), ~team ('blue'/'red'), ~role ('attacker'/'defender'), ~opponents (ids).
Arena 1,6 x 1,6 m dividida em x = 0. Bases/bandeiras em (-0,62; 0) azul e (+0,62; 0) vermelho.
Pegar a bandeira inimiga: < 0,07 m. Pontuar: < 0,14 m da própria base. Na PRÓPRIA metade, tocar um
oponente (< 0,10 m) o segura 4 s (comandos ignorados); depois ele fica imune por 1,5 s.

FASE 0: o nó sobe, assina odometrias, bandeiras e has_flag e publica (0, 0) desde o primeiro ciclo.
A estratégia entra nas fases seguintes (veja relatorio/DECISOES.md). Funciona com 4 robôs e com
-n 2 (um robô por lado, sem colega).

Regras do arquivo (formato do torneio): autossuficiente, só biblioteca padrão, numpy, rospy,
mensagens do ROS e psr_hero.utils; nenhuma leitura/escrita de arquivo; mesmo código nos dois lados;
publica SOMENTE no cmd_vel do próprio ~robot.
"""
import math  # noqa: F401

import rospy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from psr_hero.utils import V_MAX, W_MAX, clamp, pose_from_odom, wrap_angle  # noqa: F401

BASE_X = 0.62  # m, azul em -BASE_X, vermelho em +BASE_X
BASE_RADIUS = 0.14  # m
RATE_HZ = 20.0


class CaptureTheFlag:
    def __init__(self):
        rospy.init_node("ctf")
        self.robot = rospy.get_param("~robot", 0)
        self.team = rospy.get_param("~team", "blue" if self.robot == 0 else "red")
        self.role = rospy.get_param("~role", "attacker")
        self.opponents = list(rospy.get_param("~opponents", [1 if self.robot == 0 else 0]))
        self.blue = self.team == "blue"
        self.home = (-BASE_X if self.blue else BASE_X, 0.0)  # sua base e sua bandeira
        self.me = None  # (x, y, yaw)
        self.opp = {}  # id do oponente -> (x, y, yaw)
        self.enemy_flag = None  # (x, y)
        self.own_flag = None  # (x, y): longe de self.home = um oponente a levou
        self.carrying = False  # True enquanto você carrega a bandeira inimiga

        self.pub = rospy.Publisher("/hero_%d/cmd_vel" % self.robot, Twist, queue_size=1)
        rospy.Subscriber("/hero_%d/odom" % self.robot, Odometry, self.on_me, queue_size=1)
        for k in self.opponents:
            rospy.Subscriber("/hero_%d/odom" % k, Odometry, self.on_opp, callback_args=k, queue_size=1)
        enemy, own = ("red", "blue") if self.blue else ("blue", "red")
        rospy.Subscriber("/flag/%s" % enemy, PointStamped, self.on_enemy_flag, queue_size=1)
        rospy.Subscriber("/flag/%s" % own, PointStamped, self.on_own_flag, queue_size=1)
        rospy.Subscriber("/hero_%d/has_flag" % self.robot, Bool, self.on_has_flag, queue_size=1)

    def on_me(self, msg):
        self.me = pose_from_odom(msg)

    def on_opp(self, msg, k):
        self.opp[k] = pose_from_odom(msg)

    def on_enemy_flag(self, msg):
        self.enemy_flag = (msg.point.x, msg.point.y)

    def on_own_flag(self, msg):
        self.own_flag = (msg.point.x, msg.point.y)

    def on_has_flag(self, msg):
        self.carrying = bool(msg.data)

    def compute_command(self, me, opponents, enemy_flag, own_flag, carrying):
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
        if self.me is not None and self.enemy_flag is not None and self.own_flag is not None:
            try:
                v, w = self.compute_command(self.me, list(self.opp.values()), self.enemy_flag,
                                            self.own_flag, self.carrying)
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
    CaptureTheFlag().run()
