#!/usr/bin/env python3
"""SPARRING soccer_nyx: retranca com os três na linha do gol e contra-ataque ocasional (simula o Nyx).
Só para teste local.

`blue` (robôs 0, 1, 2) ataca +x; `red` (robôs 3, 4, 5) ataca -x. Campo 1,8 x 1,2 m, centro (0, 0), gols de
0,40 m no meio dos lados curtos. O índice 0 de self.ids é o goleiro (ordem de nascimento).

v1 (Fase 2). Papéis (os mesmos nomes do relatório), reavaliados a cada ciclo:
  GOLEIRO   fica na linha x = -0,80*attack, na altura da bola (|y| <= 0,16).
  ATACANTE  o jogador de linha mais perto da bola (com histerese de 20%): vai ao ponto de preparação atrás da
            bola, na linha bola -> gol adversário, e então atravessa a bola à velocidade máxima. Se estiver do
            lado errado da bola, contorna por fora (nunca empurra a bola para o próprio gol).
  APOIO     o outro jogador de linha: espera recuado em (-0,35*attack, 0).
Todos têm detector de travamento (ré curta com giro). O placar mudando (gol) reinicia a memória.

Regras do arquivo (formato do torneio): autossuficiente, só biblioteca padrão, numpy, rospy, mensagens do ROS
e psr_hero.utils; nenhuma leitura/escrita de arquivo; mesmo código nos dois lados; publica SOMENTE nos três
robôs do próprio time; o nó nunca morre.
"""
import json
import math

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from psr_hero.utils import V_MAX, W_MAX, clamp, pose_from_odom, wrap_angle

FIELD_L, FIELD_W = 1.8, 1.2
GOAL_W = 0.40
TEAMS = {"blue": (0, +1), "red": (3, -1)}  # primeiro id do time, sentido de ataque em x
RATE_HZ = 20.0
KW = 5.0  # ganho de giro do controlador base
KEEPER_X = 0.80  # m, a linha do goleiro fica a KEEPER_X do centro, do lado do próprio gol
KEEPER_Y = 0.16  # m, o goleiro não passa disso (o gol vai até 0,20)
PREP = 0.12  # m, distância do ponto de preparação atrás da bola
FLANK = 0.16  # m, desvio lateral ao contornar a bola
ALIGN_ERR = 0.26  # rad (~15 graus) máximo de erro de alinhamento para chutar
ALIGN_LAT = 0.02  # m, desvio lateral máximo para chutar
SWITCH_GAIN = 0.8  # um novo atacante só assume se o tempo de chegada dele for < 80% do atual
SUPPORT_X = 0.35  # m, o apoio espera em x = -SUPPORT_X*attack
MARGIN = 0.05  # m, os alvos ficam a pelo menos isso das paredes
CLEAR_DEPTH = 0.74  # m, bola a mais disso do centro, em direção ao NOSSO gol: tira de lado em vez de ir ao gol
R_ORB = 0.16  # m, raio da órbita ao redor da bola quando o robô está do lado errado dela
ORB_STEP = 0.8  # rad, quanto avançar ao longo da órbita a cada ciclo
GK_CLEAR_DEPTH = 0.78  # m, bola mais funda que isso na nossa área: o goleiro também disputa a bola (a limpa de lado)
IDLE_S = 1.0  # s com comando praticamente zero = parado sem motivo (rede de segurança)
T_STUCK = 1.5  # s com comando alto e robô parado = travado
STUCK_BACK = 0.4  # s de ré ao destravar

GOLEIRO, ATACANTE, APOIO = "GOLEIRO", "ATACANTE", "APOIO"


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


def in_field(p, margin=MARGIN):
    return (clamp(p[0], -FIELD_L / 2 + margin, FIELD_L / 2 - margin),
            clamp(p[1], -FIELD_W / 2 + margin, FIELD_W / 2 - margin))


class Stuck:
    """Detector de travamento: |v_cmd| > 0,1 e o robô parado por mais de T_STUCK s -> ré curta com giro."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.anchor = None  # (x, y, t) da última posição em que o robô se mexeu
        self.until = None
        self.cool = 0.0
        self.turn = 1.0

    def update(self, now, pos, v_cmd):
        """Devolve (v, w) de recuperação, ou None se não há travamento."""
        if self.until is not None:
            if now < self.until:
                return -0.12, 2.5 * self.turn
            self.until, self.anchor, self.cool = None, None, now + 1.0
        if abs(v_cmd) <= 0.1 or now < self.cool:
            self.anchor = None
            return None
        if self.anchor is None or math.hypot(pos[0] - self.anchor[0], pos[1] - self.anchor[1]) > 0.012:
            self.anchor = (pos[0], pos[1], now)
            return None
        if now - self.anchor[2] > T_STUCK:
            self.until, self.anchor, self.turn = now + STUCK_BACK, None, -self.turn
            return -0.12, 2.5 * self.turn
        return None


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
        self.striker = None  # id do atacante atual
        self.stuck = dict((i, Stuck()) for i in self.ids)
        self.score = None
        self.roles = {}
        self.idle_since = None
        self.unstick_until = 0.0

        self.pubs = {i: rospy.Publisher("/hero_%d/cmd_vel" % i, Twist, queue_size=1) for i in self.ids}
        for i in range(6):
            rospy.Subscriber("/hero_%d/odom" % i, Odometry, self.on_odom, callback_args=i, queue_size=1)
        rospy.Subscriber("/ball/odom", Odometry, self.on_ball, queue_size=1)
        rospy.Subscriber("/game/state", String, self.on_state, queue_size=1)

    def on_odom(self, msg, i):
        self.poses[i] = pose_from_odom(msg)

    def on_ball(self, msg):
        p = msg.pose.pose.position
        self.ball = (p.x, p.y)

    def on_state(self, msg):
        """Gol (placar mudou): zera o atacante e os detectores de travamento."""
        try:
            g = json.loads(msg.data)
            score = (g.get("blue"), g.get("red"))
        except (ValueError, TypeError, AttributeError):
            return
        if self.score is not None and score != self.score:
            self.striker = None
            for s in self.stuck.values():
                s.reset()
        self.score = score

    # --- papéis -----------------------------------------------------------------------

    def eta(self, pose, ball):
        """Tempo aproximado de chegada à bola (s): distância/V_MAX mais o giro."""
        d = math.hypot(ball[0] - pose[0], ball[1] - pose[1])
        turn = abs(wrap_angle(math.atan2(ball[1] - pose[1], ball[0] - pose[0]) - pose[2]))
        return d / V_MAX + turn / W_MAX

    def pick_striker(self, poses, ball):
        """Atacante = quem chega primeiro à bola, com histerese de 20%. O goleiro só entra na disputa quando
        a bola está funda na nossa área (GK_CLEAR_DEPTH): ali o atacante de linha não consegue passar por ele."""
        cands = list(self.ids[1:])
        if -self.attack * ball[0] > GK_CLEAR_DEPTH:
            cands.append(self.ids[0])
        best = min(cands, key=lambda i: self.eta(poses[i], ball))
        if self.striker in cands and best != self.striker:
            if self.eta(poses[best], ball) >= SWITCH_GAIN * self.eta(poses[self.striker], ball):
                best = self.striker  # histerese: só troca se o novo for pelo menos 20% mais rápido
        if best != self.striker:
            rospy.loginfo("atacante: %s -> %s" % (self.striker, best))
        self.striker = best
        return best

    def keeper(self, me, ball):
        x_line = -KEEPER_X * self.attack
        v, w, _ = go_to(me, (x_line, clamp(ball[1], -KEEPER_Y, KEEPER_Y)), d_slow=0.03, reverse=True)
        return v, w

    def push_dir(self, ball):
        """Direção (unitária) em que o atacante deve empurrar a bola.

        Normal: da bola para o gol adversário. Com a bola colada no NOSSO fundo (CLEAR_DEPTH), ir ao gol
        exigiria ficar atrás dela, fora do campo, e empurrar para o nosso gol seria arriscado: então
        limpa para a lateral (ao longo de y, para o lado onde a bola já está) e um pouco para frente.
        """
        depth = -self.attack * ball[0]  # > 0: quanto a bola está do lado do nosso gol
        if depth > CLEAR_DEPTH:
            ux, uy = 0.5 * self.attack, (1.0 if ball[1] >= 0.0 else -1.0)
        else:
            ux, uy = self.attack * (FIELD_L / 2.0) - ball[0], -ball[1]
        n = math.hypot(ux, uy)
        return (ux / n, uy / n) if n > 1e-6 else (float(self.attack), 0.0)

    def orbit_point(self, me, ball, ux, uy):
        """Ponto numa órbita de raio R_ORB ao redor da bola, um passo na direção do ponto 'atrás da bola'.

        Sempre difere da posição atual (o robô nunca fica parado) e prefere o sentido cujo ponto cabe
        no campo, para não ser cortado contra a parede.
        """
        ang = math.atan2(me[1] - ball[1], me[0] - ball[0])
        want = math.atan2(-uy, -ux)  # direção, a partir da bola, do ponto 'atrás'
        diff = wrap_angle(want - ang)
        best = None
        for step in (clamp(diff, -ORB_STEP, ORB_STEP),
                     clamp(diff - math.copysign(2.0 * math.pi, diff), -ORB_STEP, ORB_STEP)):
            p = (ball[0] + R_ORB * math.cos(ang + step), ball[1] + R_ORB * math.sin(ang + step))
            q = in_field(p)
            err = math.hypot(p[0] - q[0], p[1] - q[1])
            if err < 0.01:
                return q
            if best is None or err < best[0]:
                best = (err, q)
        return best[1]

    def striker_cmd(self, me, ball):
        ux, uy = self.push_dir(ball)
        rx, ry = me[0] - ball[0], me[1] - ball[1]
        along = rx * ux + ry * uy  # < 0: estamos atrás da bola, em relação à direção de empurrão
        lateral = -rx * uy + ry * ux  # desvio lateral em relação à linha de empurrão
        if along > -0.06:  # do lado errado da bola: dá a volta por uma órbita, sem tocá-la
            v, w, _ = go_to(me, self.orbit_point(me, ball, ux, uy), d_slow=0.03)
            return v, w
        err = wrap_angle(math.atan2(ball[1] - me[1], ball[0] - me[0]) - me[2])
        if abs(lateral) < ALIGN_LAT + 0.03 and abs(err) < ALIGN_ERR and math.hypot(rx, ry) < 0.22:
            through = (ball[0] + 0.10 * ux, ball[1] + 0.10 * uy)  # atravessa a bola
            v, w, _ = go_to(me, through, d_slow=1e-3)
            return v, w
        prep = in_field((ball[0] - PREP * ux, ball[1] - PREP * uy))
        if math.hypot(me[0] - prep[0], me[1] - prep[1]) < 0.03:
            # Chegou ao ponto de preparação (que pode ter sido cortado pela parede, com a bola encostada nela,
            # e então o desvio lateral fica acima da tolerância). Sem esta regra go_to devolveria (0, 0), por
            # estar em cima do alvo, e o robô ficaria parado para sempre.
            if abs(err) < ALIGN_ERR:  # já encara a bola: chuta mesmo com o desvio lateral
                through = (ball[0] + 0.10 * ux, ball[1] + 0.10 * uy)
                v, w, _ = go_to(me, through, d_slow=1e-3)
                return v, w
            return 0.0, clamp(KW * err, -W_MAX, W_MAX)  # senão gira no lugar para encarar a bola
        v, w, _ = go_to(me, prep, d_slow=0.03)
        return v, w

    def striker_safe(self, me, ball):
        """striker_cmd com rede de segurança: parado sem motivo por IDLE_S -> vai direto na bola por 0,8 s."""
        now = rospy.get_time()
        if now < self.unstick_until:
            v, w, _ = go_to(me, ball, d_slow=1e-3)
            return v, w
        v, w = self.striker_cmd(me, ball)
        if abs(v) < 0.02 and abs(w) < 0.3:
            if self.idle_since is None:
                self.idle_since = now
            elif now - self.idle_since > IDLE_S:
                self.idle_since, self.unstick_until = None, now + 0.8
                rospy.logwarn_throttle(2.0, "atacante parado sem motivo: empurra direto na bola")
        else:
            self.idle_since = None
        return v, w

    def support(self, me):
        v, w, _ = go_to(me, in_field((-SUPPORT_X * self.attack, 0.0)), d_slow=0.04, reverse=True)
        return v, w

    def park(self, me, target, yaw):
        """Vai ao ponto e, chegando, gira para ficar virado para o campo (yaw)."""
        v, ww, dist = go_to(me, target, d_slow=0.02, reverse=True)
        if dist > 0.015:
            return v, ww
        return 0.0, clamp(KW * wrap_angle(yaw - me[2]), -W_MAX, W_MAX)

    def compute_commands(self, poses, ball):
        """Retranca: três robôs espaçados uniformemente na linha do gol, virados para o campo. O do meio
        (índice 1) sai para chutar só quando a bola está no campo de ataque, e volta quando ela recua."""
        x = -0.858 * self.attack  # colados na linha do gol (o centro a 0,042 m do fundo)
        face = 0.0 if self.attack > 0 else math.pi
        ys = (0.0, 0.1185, -0.1185)  # 3 x 7,4 cm deixam folgas de ~4,45 cm: menos que a bola (4,28 cm)
        out = []
        for k, i in enumerate(self.ids):
            me = poses[i]
            if k == 1 and self.attack * ball[0] > 0.25:
                self.roles[i] = ATACANTE
                v, ww = self.striker_cmd(me, ball)
            else:
                self.roles[i] = GOLEIRO if k == 0 else APOIO
                v, ww = self.park(me, (x, ys[k]), face)
            out.append((v, ww))
        return out

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
                    now = rospy.get_time()
                    cmds = []
                    for i, (v, w) in zip(self.ids, out):
                        rec = self.stuck[i].update(now, self.poses[i], v)
                        if rec is not None:
                            rospy.loginfo_throttle(2.0, "robô %d travado: ré curta com giro" % i)
                        cmds.append(rec if rec is not None else (float(v), float(w)))
                else:
                    rospy.logwarn_throttle(2.0, "compute_commands devolveu %d comandos" % len(out))
            except Exception as exc:  # o nó nunca morre: um erro vira (0, 0)
                rospy.logwarn_throttle(2.0, "compute_commands falhou: %r" % (exc,))
                cmds = [(0.0, 0.0)] * 3
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
