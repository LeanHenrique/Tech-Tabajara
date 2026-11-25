#!/usr/bin/env python3
"""Sumô v4 "detecta e ancora".

Estados novos, usados só com a âncora ativa: CONTRA_CARGA, VOLTAR (ao centro) e ANCORA.
Joga exatamente como a v2 até perceber que o adversário circula de RÉ (assinatura do R2D2). A partir daí, e até o fim
da partida, ancora no centro: gira parado para ficar alinhado com ele (frente ou traseira, o corpo é um cilindro),
contra-carrega quando ele se aproxima (para chegar ao impacto com velocidade) e não o persegue até a borda.

Sumô (RTech Tabajara), um processo por robô. Parâmetros: ~robot, ~opponent.

v2 (Fase 4). Estados (os mesmos nomes do relatório):
  ENGAJAR  encara o adversário e empurra de frente à velocidade máxima. É também o estado EM CONTATO.
  BORDA    se o centro passa de R_SAFE e NÃO há contato, volta para o centro (com ré, se preciso).

Correção da v2 (medida, ver relatorio/DECISOES.md): na v1 a BORDA valia mesmo em contato. Quando o par chegava perto
da borda do adversário (e a nossa, r > R_SAFE), a ré para o centro tirava o nosso empurrão e o adversário nos levava
até o nosso lado: 3 derrotas em 12 rodadas contra o sumo_dodger, todas assim. Com as duas forças saturadas e
opostas a força líquida é zero e o par mantém a velocidade que tem; recuar deixa só a força dele. Em contato, a
resposta certa é manter o empurrão.

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
R_SAFE_HIST = 0.04  # m, histerese: BORDA só termina abaixo de R_SAFE - R_SAFE_HIST
RE_MIN = 0.08  # m/s, velocidade de ré do adversário que conta como "dando ré"
RE_ATIVA = 1.2  # s de ré acumulados para ativar a âncora
RE_ESQUECE = 0.5  # quanto o acumulado decai por segundo sem ré
RE_DIST = 0.12  # m, abaixo disso não conta (empurrado em contato também anda para trás)
R_ANCORA = 0.06  # m, dentro disso do centro só gira
CARGA_DIST = 0.22  # m, contra-carrega se ele se aproxima a menos disso
CARGA_PERTO = 0.14  # m, contra-carrega de qualquer jeito
CARGA_RMAX = 0.20  # m, não sai atrás dele além deste raio
CONTACT_DIST = 0.10  # m, a menos disso entre os centros há contato (2 raios = 0,074): não recua
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
        self.ancora = False  # latched até o fim da partida
        self.re_score = 0.0  # s de ré do adversário, com esquecimento
        self.opp_ant = None  # (t, pose) para medir a ré
        self.dist_ant = None  # (t, dist)
        self.fecha = 0.0  # velocidade de aproximação filtrada, m/s

        self.pub = rospy.Publisher("/hero_%d/cmd_vel" % self.robot, Twist, queue_size=1)
        rospy.Subscriber("/hero_%d/odom" % self.robot, Odometry, self.on_me, queue_size=1)
        rospy.Subscriber("/hero_%d/odom" % self.opponent, Odometry, self.on_opp, queue_size=1)
        rospy.Subscriber("/game/state", String, self.on_state, queue_size=1)

    def on_me(self, msg):
        self.me = pose_from_odom(msg)
        self.speed = msg.twist.twist.linear.x

    def on_opp(self, msg):
        self.opp = pose_from_odom(msg)
        self.observar(self.opp, rospy.get_time())

    def observar(self, opp, now):
        """Mede se o adversário anda de ré (velocidade projetada no heading dele) fora de contato."""
        if self.opp_ant is None:
            self.opp_ant = (now, opp)
            return
        t0, p0 = self.opp_ant
        dt = now - t0
        if dt < 0.08:
            return
        self.opp_ant = (now, opp)
        if dt > 0.5 or self.me is None or math.hypot(opp[0] - self.me[0], opp[1] - self.me[1]) < RE_DIST:
            return
        vl = ((opp[0] - p0[0]) * math.cos(opp[2]) + (opp[1] - p0[1]) * math.sin(opp[2])) / dt
        if abs(vl) > 0.3:  # teleporte do reinício de rodada
            return
        if vl < -RE_MIN:
            self.re_score = min(self.re_score + dt, 3.0)
        else:
            self.re_score = max(self.re_score - RE_ESQUECE * dt, 0.0)
        if not self.ancora and self.re_score >= RE_ATIVA:
            self.ancora = True
            rospy.loginfo("ANCORA ativada: adversario circulando de re")

    def on_state(self, msg):
        """Detecta o reinício de rodada (round_time volta a ~0) e zera a máquina de estados."""
        try:
            rt = float(json.loads(msg.data).get("round_time"))
        except (ValueError, TypeError, AttributeError):
            return
        if self.round_time is not None and rt < self.round_time - 1.0:
            self.set_estado(ENGAJAR)
            # nova rodada: zera as medidas, mas mantém a âncora (o adversário é o mesmo)
            self.opp_ant, self.re_score, self.dist_ant, self.fecha = None, 0.0, None, 0.0
        self.round_time = rt

    def set_estado(self, novo):
        if novo != self.estado:
            rospy.loginfo("estado: %s -> %s" % (self.estado, novo))
            self.estado = novo

    def ancorado(self, me, opp, now):
        r = math.hypot(me[0], me[1])
        dx, dy = opp[0] - me[0], opp[1] - me[1]
        dist = math.hypot(dx, dy)
        if self.dist_ant is not None and now > self.dist_ant[0]:
            self.fecha = 0.7 * self.fecha + 0.3 * (self.dist_ant[1] - dist) / (now - self.dist_ant[0])
        self.dist_ant = (now, dist)

        perto = dist < CARGA_PERTO or (dist < CARGA_DIST and self.fecha > 0.05)
        if dist < CONTACT_DIST or (perto and r < CARGA_RMAX):
            self.set_estado("CONTRA_CARGA")  # empurra com a frente ou com a traseira, o que estiver alinhado
            v, w, _ = go_to(me, (opp[0], opp[1]), d_slow=1e-6, reverse=True)
            return v, w
        if r > R_ANCORA:
            self.set_estado("VOLTAR")
            v, w, _ = go_to(me, (0.0, 0.0), reverse=True)
            return v, w
        self.set_estado("ANCORA")  # parado no centro, girando para ficar alinhado
        err = wrap_angle(math.atan2(dy, dx) - me[2])
        if abs(err) > math.pi / 2.0:
            err = wrap_angle(err - math.pi)
        return 0.0, clamp(KW * err, -W_MAX, W_MAX)

    def compute_command(self, me, opp, speed, now):
        """Devolve (linear_x, angular_z)."""
        if self.ancora:
            return self.ancorado(me, opp, now)
        r = math.hypot(me[0], me[1])
        contato = math.hypot(opp[0] - me[0], opp[1] - me[1]) < CONTACT_DIST
        if self.estado == BORDA and (r < R_SAFE - R_SAFE_HIST or contato):
            self.set_estado(ENGAJAR)  # em contato nunca recua: manter o empurrão
        elif self.estado != BORDA and r > R_SAFE and not contato:
            self.set_estado(BORDA)

        if self.estado == BORDA:
            # volta para o centro; com ré se o centro está atrás (não vira as costas para o adversário à toa)
            v, w, _ = go_to(me, (0.0, 0.0), reverse=True)
            return v, w
        v, w, _ = go_to(me, (opp[0], opp[1]), d_slow=1e-6)  # sem desacelerar: empurra à velocidade máxima
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
