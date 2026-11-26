#!/usr/bin/env python3
"""Rouba-bandeira (RTech Tabajara): um processo por robô.

Parâmetros: ~robot (id), ~team ('blue'/'red'), ~role ('attacker'/'defender'), ~opponents (ids).
Arena 1,6 x 1,6 m dividida em x = 0. Bases/bandeiras em (-0,62; 0) azul e (+0,62; 0) vermelho.
Pegar a bandeira inimiga: < 0,07 m. Pontuar: < 0,14 m da própria base. Na PRÓPRIA metade, tocar um
oponente (< 0,10 m) o segura 4 s (comandos ignorados); depois ele fica imune por 1,5 s.

v3. Estados (os mesmos nomes do relatório):
  defensor:  PRONTO (sem invasor: espera entre a bandeira e o meio, de frente para a ameaça),
             INTERCEPTAR (invasor que pode ser segurado: vai nele; o toque o segura),
             EMPURRAR_PRESO (invasor preso ou imune: empurra-o para a lateral da nossa metade e, perto de
             ele ser solto, fica encostado entre ele e o que ele quer: a bandeira ou, se for o portador, o meio),
             PERSEGUIR (portador solto/imune na nossa metade: vai colado nele).
  atacante:  IR_BANDEIRA (ninguém guardando: vai à bandeira desviando), CONTORNAR (defensor perto mas não em
             cima da bandeira: passa ao lado dele até o ponto de pegada livre), APROXIMAR (defensor em cima da
             bandeira: encosta nele para começar o ciclo), PRESO, FURAR (imune: direto na bandeira, sem desviar),
             ANCORAR (imune, com a bandeira e o defensor colado: fica sobre a base inimiga), VOLTAR (com a
             bandeira: pelo caminho mais curto até o meio e depois à base), AJUDAR (na nossa metade, segura um
             invasor que passa perto).

O que mudou da v2 (medido no simulador local, 5 min por partida, os dois lados):
  * Brecha do juiz 1: pegar a bandeira NÃO exige estar solto. Um portador segurado a < 0,07 m da base inimiga
    pega a bandeira de novo no mesmo instante e sai dos 4 s carregando e imune. A v2 entrava em PERSEGUIR e
    EMPURRAVA esse portador preso até o meio (era daí que vinham os gols da v2 x v2). A v3 nunca empurra um
    portador preso para o meio; o ataque da v3 usa essa brecha contra quem faz isso (ANCORAR).
  * Brecha 2: um robô preso só resiste ao empurrão com força proporcional à velocidade (kv), então anda
    ~0,1 m/s quando empurrado. A defesa da v3 empurra o invasor preso para longe da bandeira e do meio.
  * Defesa intercepta logo depois do meio: solto, o invasor só tem 1,5 s de imunidade (0,3 m) e não alcança a
    bandeira antes de ser segurado de novo.
  * Imune o atacante vai direto (sem repulsão): cada ciclo preso -> imune avança ~0,3 m em vez de ficar
    prensado na parede.
  * Imunidade de todos os robôs vem do `held` do /game/state, no relógio do próprio jogo ("t").

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
RATE_HZ = 20.0
KW = 5.0  # ganho de giro do controlador base
HOLD_S = 4.0  # s segurado (juiz)
IMUNE_S = 1.5  # s de imunidade depois de ser solto (juiz)
CONTATO = 0.074  # m, dois robôs encostados (2 x raio 0,037)
FLAG_MOVED = 0.05  # m, bandeira própria a mais disso da base = foi levada
R_REP = 0.22  # m, raio de repulsão dos oponentes
REP_GAIN = 2.0
T_STUCK = 1.5  # s com comando alto e robô parado = travado
STUCK_BACK = 0.4  # s de ré ao destravar

# defensor
PRONTO_AVANCO = 0.20  # m, posição de espera: da bandeira em direção ao meio
PRONTO_Y = 0.5  # fração do y da ameaça que o defensor acompanha na espera
MEIO_MARGEM = 0.03  # m, o defensor não persegue além disto do meio (fica na própria metade)
REPOS_TOL = 0.03  # m, fora disto da posição de empurrar o defensor se reposiciona antes
DEPOSITO_X = 0.45  # m, |x| do 'depósito' para onde empurramos o invasor preso (longe da bandeira e do meio)
DEPOSITO_Y = 0.55  # m, |y| do depósito (lado em que o invasor já está)
LADO_ANTES = 1.0  # s antes de soltá-lo, paramos de empurrar e bloqueamos o caminho dele

# atacante
PLANTADO_DIST = 0.20  # m, defensor inimigo a menos disso da bandeira = guardando a bandeira
SOBRE_BANDEIRA = 0.05  # m, guardando e a menos disso = em cima dela (só o ciclo de imunidade passa)
CONTORNO_R = 0.14  # m, raio de repulsão ao contornar um defensor que não está em cima da bandeira
FURAR_LADO = 0.06  # m, imune: mira a bandeira deslocada para o lado livre do defensor
AJUDA_RAIO = 0.15  # m, invasor a menos disso do atacante (na nossa metade): o atacante o segura
ANCORA_PERTO = 0.13  # m, defensor inimigo a menos disso = colado em nós
ANCORA_RAIO = 0.12  # m, só ancoramos se ainda estamos perto da base inimiga
ANCORA_TOL = 0.04  # m, ancorado: a menos disso da base inimiga só gira

PRONTO, INTERCEPTAR, EMPURRAR_PRESO, PERSEGUIR = "PRONTO", "INTERCEPTAR", "EMPURRAR_PRESO", "PERSEGUIR"
IR_BANDEIRA, CONTORNAR, APROXIMAR, PRESO, FURAR, ANCORAR, VOLTAR, AJUDAR = (
    "IR_BANDEIRA", "CONTORNAR", "APROXIMAR", "PRESO", "FURAR", "ANCORAR", "VOLTAR", "AJUDAR")


def go_to(me, target, v_max=V_MAX, kw=KW, d_slow=0.05, reverse=False):
    """Controlador base 'ir até um ponto'. Devolve (v, w, distância).

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


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def contorno(me, alvo, obst, r):
    """Ponto intermediário para passar ao lado de `obst` a pelo menos r (o campo repulsivo sozinho trava quando o
    obstáculo está exatamente no caminho). Se o segmento me -> alvo passa a menos de r de obst, devolve o ponto ao
    lado dele, no lado em que já estamos; senão devolve o próprio alvo."""
    ax, ay = alvo[0] - me[0], alvo[1] - me[1]
    n = math.hypot(ax, ay)
    if n < 1e-6:
        return alvo
    ax, ay = ax / n, ay / n
    ox, oy = obst[0] - me[0], obst[1] - me[1]
    along = ox * ax + oy * ay
    lateral = -ox * ay + oy * ax  # > 0: obstáculo à esquerda do caminho
    if along <= 0.0 or along >= n or abs(lateral) >= r:
        return alvo
    lado = -1.0 if lateral > 0.0 else 1.0  # passa pelo lado oposto ao obstáculo
    return (obst[0] - ay * lado * r * 1.3, obst[1] + ax * lado * r * 1.3)


class Stuck:
    """Detector de travamento: |v_cmd| > 0,1 e o robô parado por mais de T_STUCK s -> ré curta com giro."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.anchor = None
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
        self.carrying = False
        self.estado = PRONTO if self.role == "defender" else IR_BANDEIRA
        self.stuck = Stuck()
        # relógio e prisões, todos no tempo do jogo ("t" do /game/state)
        self.game_t = None
        self.round_time = None
        self.held = {}  # id -> preso agora
        self.immune_until = {}  # id -> fim da imunidade (tempo do jogo)
        self.held_until = {}  # id -> quando será solto (tempo do jogo)

        self.pub = rospy.Publisher("/hero_%d/cmd_vel" % self.robot, Twist, queue_size=1)
        rospy.Subscriber("/hero_%d/odom" % self.robot, Odometry, self.on_me, queue_size=1)
        for k in self.opponents:
            rospy.Subscriber("/hero_%d/odom" % k, Odometry, self.on_opp, callback_args=k, queue_size=1)
        enemy, own = ("red", "blue") if self.blue else ("blue", "red")
        rospy.Subscriber("/flag/%s" % enemy, PointStamped, self.on_enemy_flag, queue_size=1)
        rospy.Subscriber("/flag/%s" % own, PointStamped, self.on_own_flag, queue_size=1)
        rospy.Subscriber("/hero_%d/has_flag" % self.robot, Bool, self.on_has_flag, queue_size=1)
        rospy.Subscriber("/game/state", String, self.on_state, queue_size=1)

    # --- callbacks ----------------------------------------------------------------------

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
        """Relógio do jogo, reinício de rodada e prisões/imunidades de todos os robôs."""
        try:
            g = json.loads(msg.data)
            rt = float(g.get("round_time"))
            t = float(g.get("t", rt))
            held = list(g.get("held") or [])
        except (ValueError, TypeError, AttributeError):
            return
        if self.round_time is not None and rt < self.round_time - 1.0:  # nova rodada: o juiz zera tudo
            self.stuck.reset()
            self.held, self.immune_until, self.held_until = {}, {}, {}
            self.set_estado(PRONTO if self.role == "defender" else IR_BANDEIRA)
        self.round_time = rt
        self.game_t = t
        for i, h in enumerate(held):
            h = bool(h)
            if self.held.get(i) and not h:
                self.immune_until[i] = t + IMUNE_S  # acabou de ser solto: imune por 1,5 s
            if h and not self.held.get(i):
                self.held_until[i] = t + HOLD_S  # acabou de ser segurado: solto daqui a 4 s
            if h:
                self.immune_until[i] = max(self.held_until.get(i, t), t) + IMUNE_S
            self.held[i] = h

    def set_estado(self, novo):
        if novo != self.estado:
            rospy.loginfo("estado: %s -> %s" % (self.estado, novo))
            self.estado = novo

    def ours(self, x):
        return x * self.side > 0.0

    def taggable(self, k, now):
        return not self.held.get(k, False) and now >= self.immune_until.get(k, -1.0)

    # --- defensor -----------------------------------------------------------------------

    def defend(self, me, opp, own_flag, now):
        home = self.home
        x_lim = self.side * MEIO_MARGEM  # o defensor não sai da própria metade

        def inside(p):
            x = p[0] if p[0] * self.side > MEIO_MARGEM else x_lim
            return (x, clamp(p[1], -0.76, 0.76))

        intrusos = {k: o for k, o in opp.items() if self.ours(o[0])}
        portador = None
        if dist(own_flag, home) > FLAG_MOVED and intrusos:
            portador = min(intrusos, key=lambda k: dist(intrusos[k], own_flag))

        # 1) portador solto (e imune): persegue colado; quando a imunidade acaba, o toque devolve a bandeira
        if portador is not None and not self.held.get(portador, False):
            c = intrusos[portador]
            self.set_estado(PERSEGUIR)
            lead = (c[0] - self.side * 0.06, c[1])  # ele foge para o lado dele
            v, w, _ = go_to(me, inside(lead), d_slow=1e-3)
            return v, w

        if intrusos:
            # a ameaça é o portador ou, sem ele, o invasor mais perto da nossa bandeira
            k = portador if portador is not None else min(intrusos, key=lambda j: dist(intrusos[j], home))
            t = intrusos[k]
            if self.taggable(k, now):
                self.set_estado(INTERCEPTAR)  # o toque (< 0,10 m) o segura por 4 s
                v, w, _ = go_to(me, inside(t), d_slow=1e-3)
                return v, w
            return self.empurrar_preso(me, k, t, k == portador, now, inside)

        # 3) sem invasor: espera entre a bandeira e o meio, acompanhando a ameaça mais perto
        self.set_estado(PRONTO)
        if opp:
            t = min(opp.values(), key=lambda o: abs(o[0]))
            ty = t[1]
        else:
            t, ty = None, 0.0
        spot = (home[0] - self.side * PRONTO_AVANCO, clamp(PRONTO_Y * ty, -0.35, 0.35))
        v, w, dd = go_to(me, spot, d_slow=0.04, reverse=True)
        if dd > 0.02:
            return v, w
        face = math.atan2(t[1] - me[1], t[0] - me[0]) if t is not None else (0.0 if self.blue else math.pi)
        return 0.0, clamp(KW * wrap_angle(face - me[2]), -W_MAX, W_MAX)

    def empurrar_preso(self, me, k, t, carregando, now, inside):
        """Invasor preso ou imune (não dá para segurar agora).

        Enquanto ele está preso e falta mais de LADO_ANTES s para soltar, o empurramos para o 'depósito' na lateral
        da nossa metade (longe da bandeira E do meio): preso, ele só resiste com força ~ kv*v e anda ~0,12 m/s.
        Perto de ser solto (e durante a imunidade) ficamos encostados entre ele e o que ele quer: a bandeira
        (sem a bandeira) ou o meio do campo (portador). Quando a imunidade acaba, o contato o segura de novo.
        NUNCA empurramos um portador preso em direção ao meio (era o erro da v2: o empurrão o levava até x = 0).
        """
        self.set_estado(EMPURRAR_PRESO)
        home = self.home
        quer = (0.0, t[1]) if carregando else home  # para onde ele vai quando for solto
        falta = self.held_until.get(k, now) - now  # s até ser solto (<= 0: já está solto, imune)
        if falta > LADO_ANTES:
            alvo = (self.side * DEPOSITO_X, math.copysign(DEPOSITO_Y, t[1] if abs(t[1]) > 1e-3 else 1.0))
            gx, gy = alvo[0] - t[0], alvo[1] - t[1]
            g = math.hypot(gx, gy)
            if g < 0.03:  # já está no depósito: só segura a posição entre ele e o que ele quer
                falta = 0.0
            else:
                ux, uy = gx / g, gy / g  # empurrar nesta direção
                slot = (t[0] - ux * CONTATO, t[1] - uy * CONTATO)
                if dist(me, slot) > REPOS_TOL:
                    v, w = steer(me, inside(slot), [(t[0], t[1])], r_rep=0.10, gain=3.0)
                    return v, w
                v, w, _ = go_to(me, (t[0] + ux * 0.2, t[1] + uy * 0.2), d_slow=1e-3)
                return v, w
        # bloqueio: encostado entre ele e `quer`, empurrando-o para longe de `quer`
        d = dist(t, quer)
        ux, uy = ((t[0] - quer[0]) / d, (t[1] - quer[1]) / d) if d > 1e-6 else (-self.side, 0.0)
        slot = (t[0] - ux * CONTATO, t[1] - uy * CONTATO)
        if dist(me, slot) > REPOS_TOL:
            v, w = steer(me, inside(slot), [(t[0], t[1])], r_rep=0.10, gain=3.0)
            return v, w
        v, w, _ = go_to(me, (t[0] + ux * 0.2, t[1] + uy * 0.2), d_slow=1e-3)
        return v, w

    # --- atacante -----------------------------------------------------------------------

    def attack(self, me, opp, enemy_flag, carrying, now):
        if self.held.get(self.robot, False):
            self.set_estado(PRESO)  # preso: os comandos são ignorados; não gasta o detector de travamento
            self.stuck.reset()
            return 0.0, 0.0
        imune = now < self.immune_until.get(self.robot, -1.0)
        inimigos = [o for o in opp.values() if not self.ours(o[0])]  # só esses podem nos segurar

        if carrying:
            base_inimiga = (-self.home[0], 0.0)
            colado = any(dist(o, me) < ANCORA_PERTO for o in inimigos)
            if imune and colado and dist(me, base_inimiga) < ANCORA_RAIO:
                # ANCORAR: com o defensor colado, fugir não adianta (mesma velocidade). Ficamos sobre a base
                # inimiga: quando ele nos segurar, a bandeira volta para a base, a < 0,07 m de nós, e o juiz nos
                # deixa pegá-la de novo mesmo presos. Saímos dos 4 s carregando e com mais 1,5 s de imunidade.
                self.set_estado(ANCORAR)
                v, w, dd = go_to(me, base_inimiga, d_slow=0.02, reverse=True)
                if dd > ANCORA_TOL:
                    return v, w
                # sobre a base: fica alinhado com o eixo x (de frente ou de costas para o meio, o que girar menos),
                # para sair andando (ou de ré) sem girar quando for solto
                face = 0.0 if abs(wrap_angle(me[2])) < math.pi / 2.0 else math.pi
                return 0.0, clamp(KW * wrap_angle(face - me[2]), -W_MAX, W_MAX)
            self.set_estado(VOLTAR)
            if imune and not self.ours(me[0]):
                # imune na metade deles: pelo caminho mais curto até o meio (só x), de ré se for mais rápido
                v, w, _ = go_to(me, (self.side * 0.12, me[1]), d_slow=1e-3, reverse=True)
                return v, w
            if imune:
                v, w, _ = go_to(me, self.home, d_slow=1e-3, reverse=True)
                return v, w
            v, w = steer(me, self.home, [(o[0], o[1]) for o in inimigos])
            rec = self.stuck.update(now, me, v)
            return rec if rec is not None else (v, w)

        # AJUDAR: ainda na nossa metade e com um invasor que pode ser segurado por perto, seguramos nós mesmos
        if self.ours(me[0]):
            alvos = [o for k, o in opp.items() if self.ours(o[0]) and self.taggable(k, now) and dist(o, me) < AJUDA_RAIO]
            if alvos:
                self.set_estado(AJUDAR)
                t = min(alvos, key=lambda o: dist(o, me))
                v, w, _ = go_to(me, t, d_slow=1e-3)
                return v, w

        defensor = None
        if inimigos:
            d = min(inimigos, key=lambda o: dist(o, enemy_flag))
            if dist(d, enemy_flag) < PLANTADO_DIST:
                defensor = d

        if defensor is None and not imune:
            self.set_estado(IR_BANDEIRA)  # defensor longe da bandeira: vai à bandeira desviando
            v, w = steer(me, enemy_flag, [(o[0], o[1]) for o in inimigos])
            rec = self.stuck.update(now, me, v)
            return rec if rec is not None else (v, w)

        if imune:
            # Imune ninguém nos segura: vai direto (sem repulsão). Contra um defensor que caça, cada ciclo
            # preso -> imune avança ~0,3 m (catraca) em vez de nos prender contra a parede.
            if defensor is None:
                defensor = min(inimigos, key=lambda o: dist(o, me)) if inimigos else (enemy_flag[0], enemy_flag[1] + 1.0)
            # FURAR: mira a bandeira deslocada para o lado em que o defensor NÃO está; o empurrão o tira de cima dela
            self.set_estado(FURAR)
            fx, fy = enemy_flag
            ax, ay = fx - me[0], fy - me[1]
            n = math.hypot(ax, ay) or 1.0
            px, py = -ay / n, ax / n  # perpendicular à linha robô -> bandeira
            lado = 1.0 if (defensor[0] - fx) * px + (defensor[1] - fy) * py < 0.0 else -1.0
            alvo = (fx + lado * px * FURAR_LADO, fy + lado * py * FURAR_LADO)
            v, w, _ = go_to(me, alvo, d_slow=1e-3)
            return v, w

        dfl = dist(defensor, enemy_flag)
        if dfl > SOBRE_BANDEIRA:
            # CONTORNAR: o defensor está perto mas não em cima da bandeira; existe um ponto de pegada (< 0,07 m da
            # bandeira) a mais de 0,10 m dele, do lado oposto. Vamos até lá contornando-o.
            self.set_estado(CONTORNAR)
            ux, uy = (enemy_flag[0] - defensor[0]) / dfl, (enemy_flag[1] - defensor[1]) / dfl
            alvo = contorno(me, (enemy_flag[0] + ux * 0.05, enemy_flag[1] + uy * 0.05), defensor, CONTORNO_R)
            v, w = steer(me, alvo, [(defensor[0], defensor[1])], r_rep=0.12, gain=3.0)
            rec = self.stuck.update(now, me, v)
            return rec if rec is not None else (v, w)
        self.set_estado(APROXIMAR)  # em cima da bandeira: encosta nele (será segurado: começa um novo ciclo)
        v, w, _ = go_to(me, (defensor[0], defensor[1]), d_slow=1e-3)
        return v, w

    def compute_command(self, me, opponents, enemy_flag, own_flag, carrying, now=0.0):
        """Devolve (linear_x, angular_z). `opponents` é o dicionário id -> (x, y, yaw)."""
        if not isinstance(opponents, dict):
            opponents = dict(enumerate(opponents))
        if self.role == "defender":
            return self.defend(me, opponents, own_flag, now)
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
            now = self.game_t if self.game_t is not None else rospy.get_time()
            try:
                v, w = self.compute_command(self.me, dict(self.opp), self.enemy_flag,
                                            self.own_flag, self.carrying, now)
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
