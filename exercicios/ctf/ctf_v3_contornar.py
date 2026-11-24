#!/usr/bin/env python3
"""Rouba-bandeira (RTech Tabajara): um processo por robô.

Parâmetros: ~robot (id), ~team ('blue'/'red'), ~role ('attacker'/'defender'), ~opponents (ids).
Arena 1,6 x 1,6 m dividida em x = 0. Bases/bandeiras em (-0,62; 0) azul e (+0,62; 0) vermelho.
Pegar a bandeira inimiga: < 0,07 m. Pontuar: < 0,14 m da própria base. Na PRÓPRIA metade, tocar um
oponente (< 0,10 m) o segura 4 s (comandos ignorados); depois ele fica imune por 1,5 s.

v2 (Fase 4). Estados (os mesmos nomes do relatório):
  defensor:  PLANTADO (parado sobre a bandeira, de frente para o intruso) e PERSEGUIR (quando a bandeira
             foi levada e o portador está na nossa metade); volta a PLANTADO ao cruzar x = 0.
  atacante:  IR_BANDEIRA (defensor longe da bandeira: vai à bandeira desviando dos oponentes) e, com o defensor
             inimigo plantado, o CICLO DE IMUNIDADE: APROXIMAR (encosta no defensor e é segurado 4 s), PRESO
             (comandos ignorados), EMPURRAR (solto, imune por 1,5 s: vai direto no defensor e na bandeira, sem
             desviar nem girar) e FUGA_RE (com a bandeira: de ré, de frente para o defensor); VOLTAR (com a bandeira,
             sem defensor por perto, vai à base).
  atacante contra DEFENSOR ATIVO (v3; tática do Claude contra o TuoLing): CONTORNAR (circula o defensor), RE_BANDEIRA
             (de frente para o defensor e com a bandeira atrás: dá ré até ela), CORRIDA (imune: corre à bandeira) e,
             com a bandeira e o defensor colado, FUGA_RE.

Correção da v2 (observada na liga): na v1, depois de ser segurado o atacante GIRAVA (repulsão dos oponentes e detector
de travamento) e voltava a ser segurado. Agora ele rastreia o `held` do próprio robô (do /game/state), usa a janela de
imunidade para ir direto e não aciona o detector de travamento enquanto está preso.

Medido/derivado do código do juiz (relatorio/DECISOES.md): tag 0,10 > contato 0,074 > pegada 0,07, então um
defensor exatamente sobre a bandeira só é driblado se for deslocado.

Regras do arquivo (formato do torneio): autossuficiente, só biblioteca padrão, numpy, rospy, mensagens do
ROS e psr_hero.utils; nenhuma leitura/escrita de arquivo; mesmo código nos dois lados; publica SOMENTE no
cmd_vel do próprio ~robot; o nó nunca morre; funciona com -n 2 (um robô por lado, sem colega).
"""
import json
import math

import rospy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

from psr_hero.utils import V_MAX, W_MAX, clamp, pose_from_odom, wrap_angle

BASE_X = 0.62  # m, azul em -BASE_X, vermelho em +BASE_X
BASE_RADIUS = 0.14  # m
RATE_HZ = 20.0
KW = 5.0  # ganho de giro do controlador base
HOME_TOL = 0.012  # m, dentro disso o defensor considera que está sobre a bandeira (ganho de posição ~17 s^-1)
FLAG_MOVED = 0.05  # m, bandeira própria a mais disso da base = foi levada
R_REP = 0.22  # m, raio de repulsão dos oponentes (maior que o raio de segurar, 0,10)
REP_GAIN = 2.0
T_STUCK = 1.5  # s com comando alto e robô parado = travado
STUCK_BACK = 0.4  # s de ré ao destravar

PLANTADO, PERSEGUIR, IR_BANDEIRA, VOLTAR = "PLANTADO", "PERSEGUIR", "IR_BANDEIRA", "VOLTAR"
APROXIMAR, PRESO, EMPURRAR, FUGA_RE = "APROXIMAR", "PRESO", "EMPURRAR", "FUGA_RE"
CONTORNAR, RE_BANDEIRA, CORRIDA = "CONTORNAR", "RE_BANDEIRA", "CORRIDA"
ENGAJA_DIST = 0.45  # m, defensor ativo a menos disso de nós: contorna em vez de ir reto à bandeira
R_CONTORNO = 0.24  # m, raio da órbita ao redor do defensor (maior que o raio de segurar, 0,10)
PASSO_CONTORNO = 0.8  # rad, quanto avança na órbita a cada ciclo
CONE_RE = 0.61  # rad (~35 graus): tolerância para "encara o defensor" e "bandeira atrás"
FUGA_DIST = 0.50  # m, defensor a menos disso e base do lado oposto a ele: foge de ré
IMUNE_S = 1.5  # s de imunidade depois de ser solto
PLANTADO_DIST = 0.15  # m, defensor inimigo a menos disso da bandeira = plantado: usa o ciclo de imunidade


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


def steer(me, target, avoid, r_rep=R_REP, gain=REP_GAIN):
    """Vai até target desviando dos pontos de `avoid` (campo repulsivo). Devolve (v, w)."""
    ax, ay = target[0] - me[0], target[1] - me[1]
    d = math.hypot(ax, ay)
    if d > 1e-6:
        ax, ay = ax / d, ay / d
    rx = ry = 0.0
    for o in avoid:
        dx, dy = me[0] - o[0], me[1] - o[1]
        dd = math.hypot(dx, dy)
        if 1e-6 < dd < r_rep:
            k = gain * (r_rep - dd) / r_rep / dd
            rx, ry = rx + dx * k, ry + dy * k
    point = (me[0] + (ax + rx) * 0.2, me[1] + (ay + ry) * 0.2)
    v, w, _ = go_to(me, point, d_slow=1e-3)
    return v, w


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


class CaptureTheFlag:
    def __init__(self):
        rospy.init_node("ctf")
        self.robot = rospy.get_param("~robot", 0)
        self.team = rospy.get_param("~team", "blue" if self.robot == 0 else "red")
        self.role = rospy.get_param("~role", "attacker")
        self.opponents = list(rospy.get_param("~opponents", [1 if self.robot == 0 else 0]))
        self.blue = self.team == "blue"
        self.side = -1.0 if self.blue else 1.0  # x*side > 0 é a NOSSA metade
        self.home = (-BASE_X if self.blue else BASE_X, 0.0)  # nossa base e nossa bandeira
        self.me = None  # (x, y, yaw)
        self.opp = {}  # id do oponente -> (x, y, yaw)
        self.enemy_flag = None  # (x, y)
        self.own_flag = None  # (x, y): longe de self.home = um oponente a levou
        self.carrying = False  # True enquanto você carrega a bandeira inimiga
        self.estado = PLANTADO if self.role == "defender" else IR_BANDEIRA
        self.stuck = Stuck()
        self.round_time = None
        self.held_me = False
        self.immune_until = 0.0

        self.pub = rospy.Publisher("/hero_%d/cmd_vel" % self.robot, Twist, queue_size=1)
        rospy.Subscriber("/hero_%d/odom" % self.robot, Odometry, self.on_me, queue_size=1)
        for k in self.opponents:
            rospy.Subscriber("/hero_%d/odom" % k, Odometry, self.on_opp, callback_args=k, queue_size=1)
        enemy, own = ("red", "blue") if self.blue else ("blue", "red")
        rospy.Subscriber("/flag/%s" % enemy, PointStamped, self.on_enemy_flag, queue_size=1)
        rospy.Subscriber("/flag/%s" % own, PointStamped, self.on_own_flag, queue_size=1)
        rospy.Subscriber("/hero_%d/has_flag" % self.robot, Bool, self.on_has_flag, queue_size=1)
        rospy.Subscriber("/game/state", String, self.on_state, queue_size=1)

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

    def on_state(self, msg):
        """Reinício de rodada (round_time volta a ~0) e rastreio do 'held' do próprio robô (imunidade de 1,5 s)."""
        try:
            g = json.loads(msg.data)
            rt = float(g.get("round_time"))
            held = g.get("held") or []
        except (ValueError, TypeError, AttributeError):
            return
        if self.round_time is not None and rt < self.round_time - 1.0:
            self.stuck.reset()
            self.held_me, self.immune_until = False, 0.0
            self.set_estado(PLANTADO if self.role == "defender" else IR_BANDEIRA)
        self.round_time = rt
        now_held = bool(held[self.robot]) if self.robot < len(held) else False
        if self.held_me and not now_held:
            self.immune_until = rospy.get_time() + IMUNE_S  # acabou de ser solto: imune por 1,5 s
        self.held_me = now_held

    def set_estado(self, novo):
        if novo != self.estado:
            rospy.loginfo("estado: %s -> %s" % (self.estado, novo))
            self.estado = novo

    # --- defensor ---------------------------------------------------------------------

    def defend(self, me, opponents, own_flag):
        home = self.home
        stolen = math.hypot(own_flag[0] - home[0], own_flag[1] - home[1]) > FLAG_MOVED
        carrier = None
        if stolen and opponents:
            carrier = min(opponents, key=lambda o: math.hypot(o[0] - own_flag[0], o[1] - own_flag[1]))
        if carrier is not None and carrier[0] * self.side > 0.0:
            self.set_estado(PERSEGUIR)  # o portador ainda está na nossa metade: vai atrás
            v, w, _ = go_to(me, (carrier[0], carrier[1]), d_slow=1e-3)
            return v, w
        self.set_estado(PLANTADO)
        v, w, dist = go_to(me, home, d_slow=HOME_TOL, reverse=True)  # posição rígida sobre a bandeira
        if dist > HOME_TOL:
            return v, w
        # sobre a bandeira: gira para encarar o intruso mais perto dela (ou o lado inimigo, se não há)
        intruders = [o for o in opponents if o[0] * self.side > 0.0]
        if intruders:
            t = min(intruders, key=lambda o: math.hypot(o[0] - home[0], o[1] - home[1]))
            face = math.atan2(t[1] - me[1], t[0] - me[0])
        else:
            face = 0.0 if self.blue else math.pi
        return 0.0, clamp(KW * wrap_angle(face - me[2]), -W_MAX, W_MAX)

    # --- atacante ---------------------------------------------------------------------

    def orbita(self, me, d, enemy_flag):
        """Ponto numa órbita de raio R_CONTORNO ao redor do defensor, um passo na direção em que a bandeira fica
        entre o defensor e nós... isto é, no lado da bandeira em relação ao defensor."""
        ang = math.atan2(me[1] - d[1], me[0] - d[0])
        alvo = math.atan2(enemy_flag[1] - d[1], enemy_flag[0] - d[0])  # direção defensor -> bandeira
        passo = clamp(wrap_angle(alvo - ang), -PASSO_CONTORNO, PASSO_CONTORNO)
        return (clamp(d[0] + R_CONTORNO * math.cos(ang + passo), -0.75, 0.75),
                clamp(d[1] + R_CONTORNO * math.sin(ang + passo), -0.75, 0.75))

    def attack(self, me, opponents, enemy_flag, carrying, now):
        d = None
        if opponents:
            d = min(opponents, key=lambda o: math.hypot(o[0] - enemy_flag[0], o[1] - enemy_flag[1]))  # defensor inimigo
        plantado = d is not None and math.hypot(d[0] - enemy_flag[0], d[1] - enemy_flag[1]) < PLANTADO_DIST
        if self.held_me:
            self.set_estado(PRESO)  # preso: os comandos são ignorados; não gasta o detector de travamento
            self.stuck.reset()
            return 0.0, 0.0
        if carrying:
            if d is not None:
                para_d = (d[0] - me[0], d[1] - me[1])
                para_casa = (self.home[0] - me[0], self.home[1] - me[1])
                nd, nc = math.hypot(*para_d), math.hypot(*para_casa)
                costas = nd > 1e-6 and nc > 1e-6 and (para_d[0] * para_casa[0] + para_d[1] * para_casa[1]) / (nd * nc) < -0.3
                if plantado or (nd < FUGA_DIST and costas):
                    self.set_estado(FUGA_RE)  # de ré, de frente para o defensor, afastando-se dele rumo à base
                    err = wrap_angle(math.atan2(para_d[1], para_d[0]) - me[2])
                    return -V_MAX, clamp(KW * err, -W_MAX, W_MAX)
            self.set_estado(VOLTAR)  # desvia do defensor e leva a bandeira para o nosso campo
            v, w = steer(me, self.home, [(o[0], o[1]) for o in opponents])
            rec = self.stuck.update(now, me, v)
            return rec if rec is not None else (v, w)
        if plantado:  # defensor plantado sobre a bandeira: ciclo de imunidade
            if now < self.immune_until:
                self.set_estado(EMPURRAR)  # imune: vai direto no defensor (sem desviar nem girar) e na bandeira
                v, w, _ = go_to(me, (d[0], d[1]), d_slow=1e-3)
                return v, w
            self.set_estado(APROXIMAR)  # encosta no defensor (será segurado: começa um novo ciclo)
            v, w, _ = go_to(me, (d[0], d[1]), d_slow=1e-3)
            return v, w
        if d is not None and math.hypot(d[0] - me[0], d[1] - me[1]) < ENGAJA_DIST:
            # DEFENSOR ATIVO por perto (observado na liga: ele sai ao encontro e cola no atacante)
            if now < self.immune_until:
                self.set_estado(CORRIDA)  # imune: ninguém nos segura por 1,5 s, corre direto à bandeira
                v, w, _ = go_to(me, enemy_flag, d_slow=1e-3)
                return v, w
            h = (math.cos(me[2]), math.sin(me[2]))
            para_d = (d[0] - me[0], d[1] - me[1])
            para_f = (enemy_flag[0] - me[0], enemy_flag[1] - me[1])
            nd, nf = math.hypot(*para_d), math.hypot(*para_f)
            cos_d = (h[0] * para_d[0] + h[1] * para_d[1]) / nd if nd > 1e-6 else 1.0
            cos_f = (h[0] * para_f[0] + h[1] * para_f[1]) / nf if nf > 1e-6 else -1.0
            if cos_d > math.cos(CONE_RE) and cos_f < -math.cos(CONE_RE):
                # de frente para o defensor e com a bandeira atrás: vai de ré até ela (qualquer empurrão é frontal)
                self.set_estado(RE_BANDEIRA)
                err = wrap_angle(math.atan2(para_d[1], para_d[0]) - me[2])
                return -V_MAX, clamp(KW * err, -W_MAX, W_MAX)
            self.set_estado(CONTORNAR)  # circula o defensor até ficar com a bandeira atrás e ele à frente
            v, w, _ = go_to(me, self.orbita(me, d, enemy_flag), d_slow=1e-3)
            rec = self.stuck.update(now, me, v)
            return rec if rec is not None else (v, w)
        self.set_estado(IR_BANDEIRA)  # defensor longe: vai à bandeira desviando dos oponentes
        v, w = steer(me, enemy_flag, [(o[0], o[1]) for o in opponents])
        rec = self.stuck.update(now, me, v)
        return rec if rec is not None else (v, w)

    def compute_command(self, me, opponents, enemy_flag, own_flag, carrying, now=0.0):
        """Devolve (linear_x, angular_z)."""
        if self.role == "defender":
            return self.defend(me, opponents, own_flag)
        return self.attack(me, opponents, enemy_flag, carrying, now)

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
                                            self.own_flag, self.carrying, rospy.get_time())
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
