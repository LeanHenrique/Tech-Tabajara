#!/usr/bin/env python3
"""Futebol (RTech Tabajara): um processo controla os três robôs do time. Parâmetro: ~team.

`blue` (robôs 0, 1, 2) ataca +x; `red` (robôs 3, 4, 5) ataca -x. Campo 1,8 x 1,2 m, centro (0, 0), gols de
0,40 m no meio dos lados curtos. O índice 0 de self.ids é o goleiro (ordem de nascimento).

v3. Papéis:
  GOLEIRO   é sempre o primeiro robô do time e nunca participa da seleção de atacante.
  ATACANTE  é escolhido somente entre os dois jogadores de linha, com histerese temporal.
  APOIO     ocupa uma linha de cobertura à frente do goleiro quando a bola está no nosso campo; no ataque,
            acompanha o atacante para disputar rebotes sem ocupar a mesma trajetória.
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
GOAL_MARGIN = 0.05  # m, margem em relação às traves ao escolher onde finalizar
GOAL_FOLLOW = 0.75  # quanto a finalização acompanha a altura atual da bola (0: centro; 1: mesma altura)
TEAMS = {"blue": (0, +1), "red": (3, -1)}  # primeiro id do time, sentido de ataque em x
RATE_HZ = 20.0
KW = 5.0  # ganho de giro do controlador base
KEEPER_HOME_X = 0.84  # m, posição próxima à linha do gol quando há perigo
KEEPER_SWEEP_X = 0.50  # m, posição adiantada quando a bola está longe da nossa área
KEEPER_Y = 0.15  # m, margem lateral segura dentro da boca do gol
KEEPER_TRACK_DEPTH = 0.12  # m, começa a acompanhar lateralmente quando a bola entra no nosso campo
KEEPER_PREDICT_TIME = 3.0  # s, horizonte de previsão de interceptação
PREP = 0.10  # m, distância do ponto de preparação atrás da bola
FLANK = 0.16  # m, desvio lateral ao contornar a bola
ALIGN_ERR = 0.35  # rad (~20 graus) máximo de erro de alinhamento para chutar
ALIGN_LAT = 0.05  # m, desvio lateral máximo para chutar
SWITCH_GAIN = 0.75  # o desafiante precisa ser pelo menos 25% mais rápido
SWITCH_MIN_GAIN = 0.15  # e ganhar pelo menos 0,15 s no ETA
STRIKER_MIN_HOLD = 1.5  # tempo mínimo entre trocas de atacante
STRIKER_CONFIRM = 0.6  # vantagem precisa persistir por este tempo antes da troca
SUPPORT_BACK = 0.30  # m, reserva para manter separação mínima do atacante
SUPPORT_SIDE = 0.22  # m, separação lateral do apoio
SUPPORT_ADVANCE = 0.28  # m, distância à frente da bola para oferecer opção de passe
COVER_DEPTH = 0.28  # m, a partir daqui o apoio assume uma posição de cobertura defensiva
SUPPORT_DEF_MIN = 0.18  # m, distância mínima da linha defensiva ao centro do campo
SUPPORT_DEF_MAX = 0.42  # m, distância máxima da linha defensiva ao centro do campo
SUPPORT_DEF_Y = 0.24  # m, limite lateral da posição de cobertura
GK_LOOKAHEAD = KEEPER_PREDICT_TIME
GK_VMIN = 0.03  # m/s, velocidade mínima da bola em direção ao nosso gol para prever
MARGIN = 0.05  # m, os alvos ficam a pelo menos isso das paredes
CLEAR_DEPTH = 0.74  # m, bola a mais disso do centro, em direção ao NOSSO gol: tira de lado em vez de ir ao gol
R_ORB = 0.16  # m, raio da órbita ao redor da bola quando o robô está do lado errado dela
ORB_STEP = 0.8  # rad, quanto avançar ao longo da órbita a cada ciclo
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
        self.ball_v = (0.0, 0.0)  # velocidade da bola no mundo (m/s)
        self.striker = None  # id do atacante atual
        self.pending_striker = None
        self.pending_since = None
        self.stuck = dict((i, Stuck()) for i in self.ids)
        self.score = None
        self.roles = {}
        self.last_role_signature = None
        self.idle_since = None
        self.unstick_until = 0.0
        self.start_time = rospy.get_time()
        self.striker_since = self.start_time
        self.last_status = self.start_time
        self.stuck_active = set()
        self.safe_active = False
        self.keeper_y_filtered = 0.0
        self.keeper_x_filtered = KEEPER_SWEEP_X
        self.keeper_filter_time = self.start_time
        self.last_keeper_log = 0.0

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
        self.ball_v = (msg.twist.twist.linear.x, msg.twist.twist.linear.y)

    def on_state(self, msg):
        """Gol (placar mudou): zera o atacante e os detectores de travamento."""
        try:
            g = json.loads(msg.data)
            score = (g.get("blue"), g.get("red"))
        except (ValueError, TypeError, AttributeError):
            return
        if self.score is not None and score != self.score:
            old_blue, old_red = self.score
            new_blue, new_red = score
            if self.team == "blue":
                delta_own, delta_opp = new_blue - old_blue, new_red - old_red
                own_score, opp_score = new_blue, new_red
            else:
                delta_own, delta_opp = new_red - old_red, new_blue - old_blue
                own_score, opp_score = new_red, new_blue
            event = "NOSSO" if delta_own > delta_opp else "ADVERSARIO" if delta_opp > delta_own else "PLACAR"
            rospy.loginfo("[GOL] %s | placar %d x %d | t=%.1fs | %s" %
                           (event, own_score, opp_score, rospy.get_time() - self.start_time,
                            self.snapshot()))
            self.striker = None
            self.striker_since = rospy.get_time()
            self.pending_striker = None
            self.pending_since = None
            for detector in self.stuck.values():
                detector.reset()
            self.stuck_active.clear()
        self.score = score

    def snapshot(self):
        """Resumo compacto da bola e das posições para logs de eventos."""
        ball = "bola=indisponivel" if self.ball is None else "bola=(%.2f, %.2f) v=(%.2f, %.2f)" % (
            self.ball[0], self.ball[1], self.ball_v[0], self.ball_v[1])
        robots = " | ".join("hero_%d=(%.2f, %.2f)" % (i, self.poses[i][0], self.poses[i][1])
                            for i in sorted(self.poses))
        return "%s atacante=%s | %s" % (ball, "hero_%d" % self.striker if self.striker is not None else "nenhum", robots)

    def log_status(self, now):
        if now - self.last_status < 5.0:
            return
        self.last_status = now
        rospy.loginfo("[STATUS] t=%.1fs time=%s | %s" %
                       (now - self.start_time, self.team, self.snapshot()))

    # --- papéis -----------------------------------------------------------------------

    def eta(self, pose, ball):
        """Tempo aproximado de chegada à bola (s): distância/V_MAX mais o giro."""
        d = math.hypot(ball[0] - pose[0], ball[1] - pose[1])
        turn = abs(wrap_angle(math.atan2(ball[1] - pose[1], ball[0] - pose[0]) - pose[2]))
        return d / V_MAX + turn / W_MAX

    def pick_striker(self, poses, ball):
        """Escolhe o atacante somente entre os dois jogadores de linha.

        O goleiro (self.ids[0]) é permanentemente excluído. A troca entre os jogadores
        de linha exige vantagem significativa, tempo mínimo e confirmação temporal.
        """
        now = rospy.get_time()
        cands = list(self.ids[1:])
        etas = {i: self.eta(poses[i], ball) for i in cands}
        best = min(cands, key=lambda i: etas[i])

        if self.striker not in cands:
            previous = self.striker
            self.striker = best
            self.striker_since = now
            self.pending_striker = None
            self.pending_since = None
            if best != previous:
                rospy.loginfo("[ATACANTE] %s -> hero_%d | bola=(%.2f, %.2f) | ETA=%.2fs | selecao inicial/inelegibilidade" %
                               ("nenhum" if previous is None else "hero_%d" % previous,
                                best, ball[0], ball[1], etas[best]))
            return best

        current_eta = etas[self.striker]
        if best == self.striker:
            self.pending_striker = None
            self.pending_since = None
            return self.striker

        challenger_eta = etas[best]
        gain = current_eta - challenger_eta
        enough_advantage = (challenger_eta < SWITCH_GAIN * current_eta and gain >= SWITCH_MIN_GAIN)
        if not enough_advantage:
            self.pending_striker = None
            self.pending_since = None
            return self.striker

        if self.pending_striker != best:
            self.pending_striker = best
            self.pending_since = now
            return self.striker

        held = now - self.striker_since
        confirmed = now - self.pending_since
        if held < STRIKER_MIN_HOLD or confirmed < STRIKER_CONFIRM:
            return self.striker

        previous = self.striker
        rospy.loginfo("[ATACANTE] hero_%d -> hero_%d | bola=(%.2f, %.2f) | ETA=%.2f->%.2fs | ganho=%.2fs | confirmado=%.1fs" %
                       (previous, best, ball[0], ball[1], current_eta, challenger_eta,
                        gain, confirmed))
        self.striker = best
        self.striker_since = now
        self.pending_striker = None
        self.pending_since = None
        return best

    def keeper_target(self, ball, now=None):
        """Alvo adaptativo do goleiro: avança quando a bola está longe e fecha a linha quando há perigo."""
        now = rospy.get_time() if now is None else now
        depth = -self.attack * ball[0]  # positivo quando a bola está no nosso campo
        danger = clamp((depth + 0.20) / 0.85, 0.0, 1.0)
        target_x_abs = KEEPER_SWEEP_X + (KEEPER_HOME_X - KEEPER_SWEEP_X) * danger
        target_x = -self.attack * target_x_abs

        # Longe da área, o goleiro fica centralizado e não persegue a coordenada y da bola.
        # Só acompanha a bola lateralmente quando ela entra no nosso campo ou vem em direção ao gol.
        target_y = 0.0
        vx, vy = self.ball_v
        x_line = target_x
        moving_to_goal = -self.attack * vx > GK_VMIN
        intercepting = False
        if moving_to_goal and abs(vx) > 1e-6:
            t = (x_line - ball[0]) / vx
            if 0.0 < t < GK_LOOKAHEAD:
                target_y = ball[1] + vy * t
                intercepting = True
            elif depth > KEEPER_TRACK_DEPTH:
                target_y = ball[1] * 0.55
        elif depth > KEEPER_TRACK_DEPTH:
            target_y = ball[1] * 0.55
        target_y = clamp(target_y, -KEEPER_Y, KEEPER_Y)

        # Suaviza ruído da odometria sem atrasar demais uma interceptação real.
        dt = clamp(now - self.keeper_filter_time, 0.0, 0.15)
        alpha = clamp(dt * (12.0 if intercepting else 5.0), 0.0, 1.0)
        if dt <= 0.0:
            alpha = 1.0
        self.keeper_x_filtered += alpha * (target_x_abs - self.keeper_x_filtered)
        self.keeper_y_filtered += alpha * (target_y - self.keeper_y_filtered)
        self.keeper_filter_time = now
        return -self.attack * self.keeper_x_filtered, clamp(self.keeper_y_filtered, -KEEPER_Y, KEEPER_Y)

    def keeper(self, me, ball):
        """Goleiro móvel: sai da pequena área quando não há perigo e fecha a linha ao defender."""
        target = self.keeper_target(ball)
        v, w, dist = go_to(me, target, v_max=V_MAX * 0.85, kw=3.8, d_slow=0.07, reverse=True)
        now = rospy.get_time()
        if now - self.last_keeper_log >= 3.0:
            self.last_keeper_log = now
            rospy.loginfo("[GOLEIRO] hero_%d | pos=(%.2f, %.2f) | alvo=(%.2f, %.2f) | dist=%.2f | profundidade=%.2f" %
                           (self.ids[0], me[0], me[1], target[0], target[1], dist, -self.attack * ball[0]))
        return v, w

    def goal_target(self, ball):
        """Ponto seguro dentro do gol adversário para onde a bola deve ser conduzida.

        A altura acompanha parcialmente a bola, para não gastar tempo trazendo jogadas laterais até o
        centro. A margem evita mirar nas traves e a suavização reduz mudanças bruscas de direção.
        """
        limit = GOAL_W / 2.0 - GOAL_MARGIN
        return (self.attack * (FIELD_L / 2.0), clamp(GOAL_FOLLOW * ball[1], -limit, limit))

    def push_dir(self, ball):
        """Direção (unitária) em que o atacante deve empurrar a bola.

        Na nossa metade, aponta diretamente para o ataque. Na metade adversária, aponta para um ponto
        seguro do gol, acompanhando parcialmente a altura atual da bola.
        """
        depth = -self.attack * ball[0]  # positivo na nossa metade; negativo na metade adversária
        if depth >= 0.0:
            # Na defesa e até a linha do meio: avançar em linha reta, sem mirar no gol.
            ux, uy = float(self.attack), 0.0
        else:
            # Só na metade adversária começa a orientar a condução para o gol.
            goal_x, goal_y = self.goal_target(ball)
            ux, uy = goal_x - ball[0], goal_y - ball[1]
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

    def push_cmd(self, me, ball, ux, uy):
        """Mantém o robô atravessando a bola; não reduz a velocidade ao chegar ao alvo."""
        norm = math.hypot(ux, uy)
        ux, uy = ux / max(norm, 1e-6), uy / max(norm, 1e-6)
        desired = math.atan2(uy, ux)
        err = wrap_angle(desired - me[2])
        w = clamp(KW * err, -W_MAX, W_MAX)
        # Com a bola próxima, mantém tração mesmo quando o alvo fica perto por acompanhar a bola.
        # Se estiver muito de lado/de costas, prioriza recuperar o rumo antes de acelerar.
        if abs(err) < 0.85:
            v = V_MAX * max(0.42, math.cos(err))
        else:
            v = V_MAX * max(0.0, math.cos(err))
        return v, w

    def striker_cmd(self, me, ball):
        depth = -self.attack * ball[0]
        if depth >= 0.0:
            # Na nossa metade, basta alinhar atrás da bola no eixo de ataque e continuar empurrando.
            # Não há órbita em torno da bola nem pausa para calcular um ângulo de finalização.
            dx = me[0] - ball[0]
            dy = me[1] - ball[1]
            along = dx * self.attack
            lateral = dy
            prep = in_field((ball[0] - self.attack * PREP, ball[1]))
            if along > -0.06 or abs(lateral) > ALIGN_LAT + 0.03:
                if math.hypot(me[0] - prep[0], me[1] - prep[1]) > 0.035:
                    v, w, _ = go_to(me, prep, d_slow=0.04, reverse=True)
                    return v, w
            return self.push_cmd(me, ball, float(self.attack), 0.0)

        ux, uy = self.push_dir(ball)
        rx, ry = me[0] - ball[0], me[1] - ball[1]
        along = rx * ux + ry * uy  # < 0: estamos atrás da bola, em relação à direção de empurrão
        lateral = -rx * uy + ry * ux  # desvio lateral em relação à linha de empurrão
        if along > -0.06:  # do lado errado da bola: dá a volta por uma órbita, sem tocá-la
            v, w, _ = go_to(me, self.orbit_point(me, ball, ux, uy), d_slow=0.04, reverse=True)
            return v, w
        err = wrap_angle(math.atan2(ball[1] - me[1], ball[0] - me[0]) - me[2])
        if abs(lateral) < ALIGN_LAT + 0.03 and abs(err) < ALIGN_ERR and math.hypot(rx, ry) < 0.22:
            return self.push_cmd(me, ball, ux, uy)
        prep = in_field((ball[0] - PREP * ux, ball[1] - PREP * uy))
        if math.hypot(me[0] - prep[0], me[1] - prep[1]) < 0.03:
            # Chegou ao ponto de preparação (que pode ter sido cortado pela parede, com a bola encostada nela,
            # e então o desvio lateral fica acima da tolerância). Sem esta regra go_to devolveria (0, 0), por
            # estar em cima do alvo, e o robô ficaria parado para sempre.
            if abs(err) < ALIGN_ERR:  # já encara a bola: chuta mesmo com o desvio lateral
                return self.push_cmd(me, ball, ux, uy)
            return 0.0, clamp(KW * err, -W_MAX, W_MAX)  # senão gira no lugar para encarar a bola
        v, w, _ = go_to(me, prep, d_slow=0.04, reverse=True)
        return v, w

    def striker_safe(self, me, ball):
        """striker_cmd com rede de segurança: parado sem motivo por IDLE_S -> vai direto na bola por 0,8 s."""
        now = rospy.get_time()
        if now < self.unstick_until:
            v, w, _ = go_to(me, ball, d_slow=1e-3)
            return v, w
        if self.safe_active:
            rospy.loginfo("[SEGURANCA] atacante hero_%d saiu do movimento direto | eu=(%.2f, %.2f) | bola=(%.2f, %.2f)" %
                           (self.striker, me[0], me[1], ball[0], ball[1]))
            self.safe_active = False
        v, w = self.striker_cmd(me, ball)
        if abs(v) < 0.02 and abs(w) < 0.3:
            if self.idle_since is None:
                self.idle_since = now
            elif now - self.idle_since > IDLE_S:
                self.idle_since, self.unstick_until = None, now + 0.8
                self.safe_active = True
                rospy.logwarn("[SEGURANCA] atacante hero_%d sem comando por %.1fs; indo direto para a bola | eu=(%.2f, %.2f) | bola=(%.2f, %.2f)" %
                               (self.striker, IDLE_S, me[0], me[1], ball[0], ball[1]))
        else:
            self.idle_since = None
        return v, w

    def support(self, me, ball, striker_pose):
        """Apoio separado do atacante: cobertura em perigo, opção adiantada no meio/ataque."""
        depth = -self.attack * ball[0]
        if depth > COVER_DEPTH:
            # Cobertura intermediária: não invade a pequena área nem abandona o goleiro.
            cover_depth = clamp(depth * 0.48, SUPPORT_DEF_MIN, SUPPORT_DEF_MAX)
            tgt = in_field((-self.attack * cover_depth,
                            clamp(ball[1] * 0.40, -SUPPORT_DEF_Y, SUPPORT_DEF_Y)))
        else:
            # Oferece uma linha de passe à frente da bola, deslocada lateralmente do atacante.
            side = -1.0 if ball[1] >= 0.0 else 1.0
            target_x = ball[0] + self.attack * SUPPORT_ADVANCE
            target_y = clamp(ball[1] + side * SUPPORT_SIDE, -0.42, 0.42)
            tgt = in_field((target_x, target_y))
            # Se o alvo ficar praticamente em cima do atacante, abre ainda mais a largura.
            if math.hypot(tgt[0] - striker_pose[0], tgt[1] - striker_pose[1]) < 0.18:
                tgt = in_field((target_x, ball[1] - side * (SUPPORT_SIDE + 0.12)))
        v, w, _ = go_to(me, tgt, d_slow=0.05, reverse=True)
        return v, w

    def compute_commands(self, poses, ball):
        """Devolve [(linear_x, angular_z)] x 3, na ordem de self.ids."""
        striker = self.pick_striker(poses, ball)
        support_id = self.ids[1] if striker == self.ids[2] else self.ids[2]
        signature = (self.ids[0], striker, support_id)
        if signature != self.last_role_signature:
            rospy.loginfo("[PAPEIS] goleiro=hero_%d | atacante=hero_%d | apoio=hero_%d" % signature)
            self.last_role_signature = signature

        out = []
        for k, i in enumerate(self.ids):
            me = poses[i]
            if k == 0:
                self.roles[i] = GOLEIRO
                v, w = self.keeper(me, ball)
            elif i == striker:
                self.roles[i] = ATACANTE
                v, w = self.striker_safe(me, ball)
            else:
                self.roles[i] = APOIO
                v, w = self.support(me, ball, poses[striker])
            out.append((v, w))
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
                        # A recuperação genérica dá ré e gira; isso pode tirar o goleiro da linha.
                        # O goleiro permanece sob controle exclusivo de keeper().
                        if i == self.ids[0]:
                            self.stuck[i].reset()
                            self.stuck_active.discard(i)
                            rec = None
                        else:
                            rec = self.stuck[i].update(now, self.poses[i], v)
                        if rec is not None:
                            if i not in self.stuck_active:
                                self.stuck_active.add(i)
                                anchor = self.stuck[i].anchor
                                rospy.logwarn("[TRAVAMENTO] hero_%d iniciou manobra | pos=(%.2f, %.2f) | comando=(%.2f, %.2f) | %s" %
                                              (i, self.poses[i][0], self.poses[i][1], v, w,
                                               "atacante" if i == self.striker else self.roles.get(i, "sem papel")))
                        elif i in self.stuck_active:
                            self.stuck_active.remove(i)
                            rospy.loginfo("[TRAVAMENTO] hero_%d encerrou manobra | pos=(%.2f, %.2f)" %
                                          (i, self.poses[i][0], self.poses[i][1]))
                        cmds.append(rec if rec is not None else (float(v), float(w)))
                else:
                    rospy.logwarn_throttle(2.0, "compute_commands devolveu %d comandos" % len(out))
            except Exception as exc:  # o nó nunca morre: um erro vira (0, 0)
                rospy.logwarn_throttle(2.0, "compute_commands falhou: %r" % (exc,))
                cmds = [(0.0, 0.0)] * 3
        self.log_status(rospy.get_time())
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
