#!/usr/bin/env python3
"""SPARRING sumo_tech: réplica do Tech no sumô, como o usuário observou na liga. Só para teste local.

Larga à velocidade máxima em direção ao adversário e o empurra até a borda (ficando atrás dele). Se detecta a
borda e o inimigo está atrás de si, VIRA (não dá ré) e sai da borda, voltando para o centro.

v1 (Fase 2). Estados (os mesmos nomes do relatório):
  ENGAJAR  encara o adversário e empurra de frente à velocidade máxima.
  BORDA    prioridade máxima: se o centro passa de R_SAFE, volta para o centro (com ré, se preciso).

Por que assim (medido na Fase 1, ver relatorio/DECISOES.md):
  * empurrar de frente contra um adversário que também empurra empata (H1); contra um parado, ele cai;
  * inclinar o heading não ajuda contra quem encara (H2 não confirmada): não há ARRASTAR na v1;
  * a frenagem de 0,2 m/s consome ~3,8 cm, então a margem da borda é pequena: BORDA vem antes de tudo.

Regras do arquivo (formato do torneio): autossuficiente, só biblioteca padrão, numpy, rospy, mensagens do
ROS e psr_hero.utils; nenhuma leitura/escrita de arquivo; mesmo código nos dois lados; o nó nunca morre.
"""
import json
import math

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from psr_hero.utils import V_MAX, W_MAX, clamp, pose_from_odom, wrap_angle

RING_RADIUS = 0.40  # m, perde quem tem o CENTRO além disso
R_SAFE = 0.32  # m, acima disso vale BORDA (frenagem a 0,2 m/s ~3,8 cm + latência: E3)
R_SAFE_HIST = 0.04
R_EDGE = 0.33  # m, o Tech 'detecta a borda' aqui
R_EDGE_HARD = 0.37  # m, e aqui sai da borda mesmo com o inimigo à frente  # m, histerese: BORDA só termina abaixo de R_SAFE - R_SAFE_HIST
KW = 5.0  # ganho de giro do controlador base
RATE_HZ = 20.0  # frequência do laço de controle

ENGAJAR, BORDA = "ENGAJAR", "BORDA"


def go_to(me, target, v_max=V_MAX, kw=KW, d_slow=0.05, reverse=False):
    """Controlador base 'ir até um ponto' (copiado em cada arquivo de jogo). Devolve (v, w, distância).

    err = wrap(atan2(dy, dx) - yaw); w = clamp(kw*err); v = v_max*max(0, cos err)*min(1, dist/d_slow).
    reverse=True permite ré quando o alvo está atrás (|err| > 90 graus): inverte o heading-alvo e usa v < 0.
    """
    dx, dy = target[0] - me[0], target[1] - me[1]
    dist = math.hypot(dx, dy)
    if dist < 1e-6:
        return 0.0, 0.0, dist
    err = wrap_angle(math.atan2(dy, dx) - me[2])
    sign = 1.0
    if reverse and abs(err) > math.pi / 2.0:
        err = wrap_angle(err - math.pi if err > 0 else err + math.pi)
        sign = -1.0
    w = clamp(kw * err, -W_MAX, W_MAX)
    v = sign * v_max * max(0.0, math.cos(err)) * min(1.0, dist / max(d_slow, 1e-6))
    return v, w, dist


class Sumo:
    def __init__(self):
        rospy.init_node("sumo")
        self.robot = rospy.get_param("~robot", 0)
        self.opponent = rospy.get_param("~opponent", 1)
        self.me = None  # (x, y, yaw)
        self.speed = 0.0  # velocidade linear própria, m/s
        self.opp = None  # (x, y, yaw)
        self.estado = ENGAJAR
        self.round_time = None

        self.pub = rospy.Publisher("/hero_%d/cmd_vel" % self.robot, Twist, queue_size=1)
        rospy.Subscriber("/hero_%d/odom" % self.robot, Odometry, self.on_me, queue_size=1)
        rospy.Subscriber("/hero_%d/odom" % self.opponent, Odometry, self.on_opp, queue_size=1)
        rospy.Subscriber("/game/state", String, self.on_state, queue_size=1)

    def on_me(self, msg):
        self.me = pose_from_odom(msg)
        self.speed = msg.twist.twist.linear.x

    def on_opp(self, msg):
        self.opp = pose_from_odom(msg)

    def on_state(self, msg):
        """Detecta o reinício de rodada (round_time volta a ~0) e zera a máquina de estados."""
        try:
            rt = float(json.loads(msg.data).get("round_time"))
        except (ValueError, TypeError, AttributeError):
            return
        if self.round_time is not None and rt < self.round_time - 1.0:
            self.set_estado(ENGAJAR)
        self.round_time = rt

    def set_estado(self, novo):
        if novo != self.estado:
            rospy.loginfo("estado: %s -> %s" % (self.estado, novo))
            self.estado = novo

    def compute_command(self, me, opp, speed, now):
        """Devolve (linear_x, angular_z)."""
        r = math.hypot(me[0], me[1])
        ahead = (opp[0] - me[0]) * math.cos(me[2]) + (opp[1] - me[1]) * math.sin(me[2])  # > 0: o inimigo está à frente
        if r > R_EDGE and (ahead < 0.0 or r > R_EDGE_HARD):
            self.set_estado(BORDA)  # vê a borda e o inimigo ficou atrás: vira e sai, sempre para a frente
            v, w, _ = go_to(me, (0.0, 0.0), d_slow=1e-6)
            return v, w
        self.set_estado(ENGAJAR)
        v, w, _ = go_to(me, (opp[0], opp[1]), d_slow=1e-6)  # à velocidade máxima, empurrando
        return v, w

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
