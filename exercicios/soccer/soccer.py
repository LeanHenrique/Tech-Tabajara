#!/usr/bin/env python3
"""Futebol (RTech Tabajara): um processo controla os três robôs do time. Parâmetro: ~team.

`blue` (robôs 0, 1, 2) ataca +x; `red` (robôs 3, 4, 5) ataca -x. Campo 1,8 x 1,2 m, centro (0, 0),
gols de 0,40 m no meio dos lados curtos. O índice 0 de self.ids é o goleiro (ordem de nascimento).

FASE 0: o nó sobe, assina as odometrias e a bola e publica (0, 0) nos três robôs desde o primeiro
ciclo. A estratégia entra nas fases seguintes (veja relatorio/DECISOES.md).

Regras do arquivo (formato do torneio): autossuficiente, só biblioteca padrão, numpy, rospy,
mensagens do ROS e psr_hero.utils; nenhuma leitura/escrita de arquivo; mesmo código nos dois lados.
"""
import math  # noqa: F401

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from psr_hero.utils import V_MAX, W_MAX, clamp, pose_from_odom, wrap_angle  # noqa: F401

FIELD_L, FIELD_W = 1.8, 1.2
GOAL_W = 0.40
TEAMS = {"blue": (0, +1), "red": (3, -1)}  # primeiro id do time, sentido de ataque em x
RATE_HZ = 20.0


class Soccer:
    def __init__(self):
        rospy.init_node("soccer")
        team = rospy.get_param("~team", "blue")
        if team not in TEAMS:
            rospy.logwarn("~team inválido (%r), usando blue" % (team,))
            team = "blue"
        self.team = team
        self.first, self.attack = TEAMS[team]
        self.ids = [self.first + k for k in range(3)]
        self.poses = {}  # id -> (x, y, yaw), os 6 robôs
        self.ball = None  # (x, y)

        self.pubs = {i: rospy.Publisher("/hero_%d/cmd_vel" % i, Twist, queue_size=1) for i in self.ids}
        for i in range(6):
            rospy.Subscriber("/hero_%d/odom" % i, Odometry, self.on_odom, callback_args=i, queue_size=1)
        rospy.Subscriber("/ball/odom", Odometry, self.on_ball, queue_size=1)

    def on_odom(self, msg, i):
        self.poses[i] = pose_from_odom(msg)

    def on_ball(self, msg):
        p = msg.pose.pose.position
        self.ball = (p.x, p.y)

    def compute_commands(self, poses, ball):
        """Devolve [(linear_x, angular_z)] x 3, um por robô do time. Fase 0: parados."""
        return [(0.0, 0.0)] * 3

    def publish(self, i, v, w):
        cmd = Twist()
        cmd.linear.x = clamp(v, -V_MAX, V_MAX)
        cmd.angular.z = clamp(w, -W_MAX, W_MAX)
        self.pubs[i].publish(cmd)

    def step(self):
        """Um ciclo: SEMPRE publica nos três robôs. Sem dados ou com exceção, publica (0, 0)."""
        cmds = [(0.0, 0.0)] * 3
        if self.ball is not None and len(self.poses) >= 6:
            try:
                out = list(self.compute_commands(dict(self.poses), self.ball))
                if len(out) == 3:
                    cmds = [(float(v), float(w)) for v, w in out]
                else:
                    rospy.logwarn_throttle(2.0, "compute_commands devolveu %d comandos" % len(out))
            except Exception as exc:  # o nó nunca morre: um erro vira (0, 0)
                rospy.logwarn_throttle(2.0, "compute_commands falhou: %r" % (exc,))
        for i, (v, w) in zip(self.ids, cmds):
            self.publish(i, v, w)

    def run(self):
        rate = rospy.Rate(RATE_HZ)
        for i in self.ids:
            self.publish(i, 0.0, 0.0)  # a partida só começa quando os dois times publicam
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()


if __name__ == "__main__":
    Soccer().run()
