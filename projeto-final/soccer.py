#!/usr/bin/env python3
"""Futebol (RTech Tabajara): um processo controla os três robôs do time. Parâmetro: ~team.

`blue` (robôs 0, 1, 2) ataca +x; `red` (robôs 3, 4, 5) ataca -x. Campo 1,8 x 1,2 m, centro (0, 0), gols de
0,40 m no meio dos lados curtos. O índice 0 de self.ids é o goleiro (ordem de nascimento).

v4.8. Papéis:
  GOLEIRO   fecha a linha de fundo em bolas laterais para não deixar espaço junto às traves e sai para
            rebater/tirar a bola quando ela entra pelo centro da área.
  ATACANTE  escolhido dinamicamente entre os dois jogadores de linha por ETA + histerese; conduz a bola
            continuamente com transição suave entre defesa/ataque e, na lateral, alinha exatamente atrás
            da bola antes de acelerar no eixo X.
  APOIO     na defesa forma uma segunda linha de cobertura atrás do atacante protegendo o corredor do gol;
            no ataque acompanha recuado na mesma lateral e antecipa resets na linha de fundo.
Todos os jogadores de linha têm detector de travamento. O placar mudando reinicia a memória.
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
GOAL_MARGIN = 0.05
GOAL_FOLLOW = 0.70
TEAMS = {"blue": (0, +1), "red": (3, -1)}
RATE_HZ = 20.0
KW = 5.2

# Goleiro
KEEPER_LINE_X = 0.86          # grudado na linha de fundo (parede fica em 0.90)
KEEPER_HOME_X = 0.85          # posição padrão de defesa
KEEPER_SWEEP_X = 0.72         # limite de avanço reduzido para não tomar bola nas costas
KEEPER_Y = 0.20               # fecha a trave (boca do gol vai até 0.20)
KEEPER_TRACK_DEPTH = 0.05
KEEPER_PREDICT_TIME = 2.5
KEEPER_CLEAR_DEPTH = 0.56     # só sai para rebater se a bola entrar fundo na área
KEEPER_CLEAR_EXIT = 0.46
KEEPER_CLEAR_Y = 0.24         # só sai em bolas centrais; laterais ele fecha na linha
KEEPER_MAX_OUT_X = 0.52
KEEPER_SIDELINE_Y = 0.36      # a partir daqui considera bola grudada na lateral
GK_VMIN = 0.03

# Atacante e condução contínua
PREP = 0.11
ALIGN_ERR = 0.42              # rad (~24 graus) para iniciar condução
ALIGN_LAT = 0.07              # m de desvio lateral para iniciar condução
DRIBBLE_KEEP_DIST = 0.35      # m, continua empurrando mesmo se a bola correr um pouco à frente
DRIBBLE_KEEP_LAT = 0.15       # m, tolerância lateral folgada durante a condução
DRIBBLE_KEEP_ERR = 0.85       # rad, tolerância angular durante a condução
SIDELINE_ZONE = 0.12          # faixa próxima à lateral (|y| >= 0.48)
WALL_STICK_Y = FIELD_W / 2.0 - 0.045  # 0.555 m: encosta o corpo do robô na lateral, alinhado com a bola
R_ORB = 0.15
ORB_STEP = 0.85

# Troca dinâmica de atacante (histerese)
SWITCH_ETA_MARGIN = 0.28      # s de vantagem mínima para o apoio virar atacante
SWITCH_HOLD_TIME = 0.20       # s sustentando a vantagem antes de trocar

# Apoio ativo (ataque próximo, defesa em segunda linha e espera de reset)
SUPPORT_BACK = 0.18           # m atrás da bola durante o ataque (mais recuado para pegar sobras)
SUPPORT_SIDE = 0.12           # m de deslocamento lateral (bem próximo do companheiro e da lateral)
COVER_DEPTH = 0.10
ENDLINE_STUCK_X = 0.76
ENDLINE_CORNER_Y = 0.20
RESET_WAIT_ATK_X = 0.38       # mais recuado para esperar o reset sem ter que dar a volta
RESET_WAIT_ATK_Y = 0.40       # próximo da lateral onde a bola está
RESET_WAIT_DEF_X = 0.56

# Segurança / travamento
MARGIN = 0.05
IDLE_S = 0.8
T_STUCK = 1.3
STUCK_BACK = 0.35

GOLEIRO, ATACANTE, APOIO = "GOLEIRO", "ATACANTE", "APOIO"


def go_to(me, target, v_max=V_MAX, kw=KW, d_slow=0.05, reverse=False):
    """Controlador base 'ir até um ponto'. Devolve (v, w, distância)."""
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
        self.anchor = None
        self.until = None
        self.cool = 0.0
        self.turn = 1.0

    def update(self, now, pos, v_cmd):
        if self.until is not None:
            if now < self.until:
                return -0.14, 2.8 * self.turn
            self.until, self.anchor, self.cool = None, None, now + 0.8
        if abs(v_cmd) <= 0.1 or now < self.cool:
            self.anchor = None
            return None
        if self.anchor is None or math.hypot(pos[0] - self.anchor[0], pos[1] - self.anchor[1]) > 0.012:
            self.anchor = (pos[0], pos[1], now)
            return None
        if now - self.anchor[2] > T_STUCK:
            self.until, self.anchor, self.turn = now + STUCK_BACK, None, -self.turn
            return -0.14, 2.8 * self.turn
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
        self.poses = {}
        self.ball = None
        self.ball_v = (0.0, 0.0)

        self.striker = None
        self.pending_striker = None
        self.pending_since = None
        self.dribbling = False
        self.keeper_clearing = False

        self.stuck = dict((i, Stuck()) for i in self.ids)
        self.score = None
        self.roles = {}
        self.last_role_signature = None
        self.idle_since = None
        self.unstick_until = 0.0
        self.start_time = rospy.get_time()
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
        try:
            g = json.loads(msg.data)
            score = (g.get("blue"), g.get("red"))
        except (ValueError, TypeError, AttributeError):
            return
        if self.score is not None and score != self.score:
            self.striker = None
            self.pending_striker = None
            self.pending_since = None
            self.dribbling = False
            self.keeper_clearing = False
            for detector in self.stuck.values():
                detector.reset()
            self.stuck_active.clear()
        self.score = score

    def snapshot(self):
        ball = "bola=indisponivel" if self.ball is None else "bola=(%.2f, %.2f) v=(%.2f, %.2f)" % (
            self.ball[0], self.ball[1], self.ball_v[0], self.ball_v[1])
        robots = " | ".join("hero_%d=(%.2f, %.2f)" % (i, self.poses[i][0], self.poses[i][1])
                            for i in sorted(self.poses))
        return "%s atacante=%s | %s" % (ball, "hero_%d" % self.striker if self.striker is not None else "nenhum", robots)

    def log_status(self, now):
        if now - self.last_status < 5.0:
            return
        self.last_status = now
        rospy.loginfo("[STATUS] t=%.1fs time=%s | %s" % (now - self.start_time, self.team, self.snapshot()))

    # --- seleção dinâmica de papéis ---------------------------------------------------

    def attack_eta(self, pose, ball):
        """Tempo efetivo para atacar a bola, penalizando quem está do lado errado dela."""
        ux, uy = self.push_dir(ball)
        rx, ry = pose[0] - ball[0], pose[1] - ball[1]
        d = math.hypot(rx, ry)
        turn = abs(wrap_angle(math.atan2(ball[1] - pose[1], ball[0] - pose[0]) - pose[2]))
        along = rx * ux + ry * uy
        wrong_side_penalty = 0.65 if along > 0.02 else 0.0
        return d / V_MAX + turn / W_MAX + wrong_side_penalty

    def pick_striker(self, poses, ball):
        """Escolhe o melhor atacante entre os dois jogadores de linha com histerese."""
        cands = list(self.ids[1:])
        now = rospy.get_time()

        if self.striker not in cands:
            self.striker = min(cands, key=lambda i: self.attack_eta(poses[i], ball))
            self.pending_striker = None
            self.pending_since = None
            self.dribbling = False
            return self.striker

        other = cands[0] if cands[1] == self.striker else cands[1]
        cur_eta = self.attack_eta(poses[self.striker], ball)
        oth_eta = self.attack_eta(poses[other], ball)

        margin = SWITCH_ETA_MARGIN + (0.35 if self.dribbling else 0.0)
        striker_in_goal = (self.attack * poses[self.striker][0] > FIELD_L / 2.0 and
                           self.attack * poses[self.striker][0] > self.attack * ball[0])

        if striker_in_goal or (oth_eta + margin < cur_eta):
            if self.pending_striker != other:
                self.pending_striker = other
                self.pending_since = now
            elif striker_in_goal or (now - self.pending_since >= SWITCH_HOLD_TIME):
                rospy.loginfo("[ATACANTE] hero_%d -> hero_%d | eta %.2fs vs %.2fs" %
                              (self.striker, other, cur_eta, oth_eta))
                self.striker = other
                self.pending_striker = None
                self.pending_since = None
                self.dribbling = False
        else:
            self.pending_striker = None
            self.pending_since = None

        return self.striker

    # --- goleiro (linha de fundo + saída para tirar a bola da área) -------------------

    def keeper_target(self, ball, now=None):
        now = rospy.get_time() if now is None else now
        depth = -self.attack * ball[0]
        on_sideline = abs(ball[1]) >= KEEPER_SIDELINE_Y

        if (depth > 0.05 and on_sideline) or depth > 0.65:
            target_x_abs = KEEPER_LINE_X
            if on_sideline:
                target_y = math.copysign(KEEPER_Y, ball[1])
            else:
                target_y = clamp(ball[1], -KEEPER_Y, KEEPER_Y)
            self.keeper_x_filtered = target_x_abs
            self.keeper_y_filtered = target_y
            self.keeper_filter_time = now
            return -self.attack * target_x_abs, target_y

        danger = clamp((depth + 0.20) / 0.85, 0.0, 1.0)
        target_x_abs = KEEPER_SWEEP_X + (KEEPER_HOME_X - KEEPER_SWEEP_X) * danger
        target_x = -self.attack * target_x_abs

        target_y = 0.0
        vx, vy = self.ball_v
        moving_to_goal = -self.attack * vx > GK_VMIN
        intercepting = False
        if moving_to_goal and abs(vx) > 1e-6:
            t = (target_x - ball[0]) / vx
            if 0.0 < t < KEEPER_PREDICT_TIME:
                target_y = ball[1] + vy * t
                intercepting = True
            elif depth > KEEPER_TRACK_DEPTH:
                target_y = ball[1] * 0.80
        elif depth > KEEPER_TRACK_DEPTH:
            target_y = ball[1] * 0.80
        target_y = clamp(target_y, -KEEPER_Y, KEEPER_Y)

        dt = clamp(now - self.keeper_filter_time, 0.0, 0.15)
        alpha = clamp(dt * (14.0 if intercepting else 6.0), 0.0, 1.0) if dt > 0.0 else 1.0
        self.keeper_x_filtered += alpha * (target_x_abs - self.keeper_x_filtered)
        self.keeper_y_filtered += alpha * (target_y - self.keeper_y_filtered)
        self.keeper_filter_time = now
        return -self.attack * self.keeper_x_filtered, clamp(self.keeper_y_filtered, -KEEPER_Y, KEEPER_Y)

    def keeper(self, me, ball):
        """Goleiro móvel: fecha a linha de fundo em bolas laterais e sai para rebater bolas centrais."""
        depth = -self.attack * ball[0]
        me_depth = -self.attack * me[0]
        ball_in_box = depth > KEEPER_CLEAR_DEPTH and abs(ball[1]) < KEEPER_CLEAR_Y
        ball_still_danger = depth > KEEPER_CLEAR_EXIT and abs(ball[1]) < KEEPER_CLEAR_Y + 0.06

        if ball_in_box and me_depth > KEEPER_MAX_OUT_X:
            self.keeper_clearing = True
        elif not ball_still_danger or me_depth <= KEEPER_MAX_OUT_X:
            self.keeper_clearing = False

        if self.keeper_clearing:
            behind_ball = self.attack * (me[0] - ball[0]) < -0.02
            if behind_ball:
                clear_y = math.copysign(0.35, ball[1]) if abs(ball[1]) > 0.05 else 0.0
                ux = float(self.attack)
                uy = clear_y - ball[1]
                n = math.hypot(ux, uy)
                return self.push_cmd(me, ball, ux / n, uy / n)
            else:
                retreat = in_field((-self.attack * KEEPER_LINE_X, clamp(ball[1], -KEEPER_Y, KEEPER_Y)))
                v, w, _ = go_to(me, retreat, v_max=V_MAX, kw=5.0, d_slow=0.04, reverse=True)
                return v, w

        target = self.keeper_target(ball)
        v, w, _ = go_to(me, target, v_max=V_MAX, kw=4.5, d_slow=0.04, reverse=True)
        return v, w

    # --- ataque e condução contínua ---------------------------------------------------

    def goal_target(self, ball):
        limit = GOAL_W / 2.0 - GOAL_MARGIN
        return (self.attack * (FIELD_L / 2.0), clamp(GOAL_FOLLOW * ball[1], -limit, limit))

    def push_dir(self, ball):
        """Direção de condução contínua, sem saltos bruscos entre defesa, ataque e laterais."""
        depth = -self.attack * ball[0]  # positivo na defesa, negativo no ataque
        goal_x, goal_y = self.goal_target(ball)
        gx, gy = goal_x - ball[0], goal_y - ball[1]
        gn = math.hypot(gx, gy)
        if gn > 1e-6:
            gx, gy = gx / gn, gy / gn
        else:
            gx, gy = float(self.attack), 0.0

        # Mistura gradual da defesa para o ataque (entre depth=0.35 e depth=-0.10)
        atk_blend = clamp((0.35 - depth) / 0.45, 0.0, 1.0)
        # Perto da parede lateral, reduz gradualmente a inclinação para ficar em eixo único
        side_margin = FIELD_W / 2.0 - abs(ball[1])
        side_blend = clamp((side_margin - 0.08) / 0.14, 0.0, 1.0)
        w_goal = atk_blend * side_blend

        ux = (1.0 - w_goal) * float(self.attack) + w_goal * gx
        uy = w_goal * gy
        n = math.hypot(ux, uy)
        return (ux / n, uy / n) if n > 1e-6 else (float(self.attack), 0.0)

    def orbit_point(self, me, ball, ux, uy):
        ang = math.atan2(me[1] - ball[1], me[0] - ball[0])
        want = math.atan2(-uy, -ux)
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
        """Conduz a bola continuamente em velocidade máxima, corrigindo o ângulo sem frear."""
        aim_x = ball[0] + 0.08 * ux
        aim_y = ball[1] + 0.08 * uy
        desired = math.atan2(aim_y - me[1], aim_x - me[0])
        err = wrap_angle(desired - me[2])
        w = clamp(KW * err, -W_MAX, W_MAX)
        if abs(err) < 0.90:
            v = V_MAX * max(0.65, math.cos(err))
        else:
            v = V_MAX * max(0.0, math.cos(err))
        return v, w

    def striker_cmd(self, me, ball):
        # 1. Rotina de lateral: primeiro cola atrás da bola na parede, depois anda só no eixo X
        if abs(ball[1]) >= FIELD_W / 2.0 - SIDELINE_ZONE:
            wall_sign = 1.0 if ball[1] >= 0.0 else -1.0
            lane_y = clamp(ball[1], -WALL_STICK_Y, WALL_STICK_Y)
            along_x = (me[0] - ball[0]) * self.attack
            lat_err = abs(me[1] - lane_y)
            x_heading = 0.0 if self.attack > 0 else math.pi

            # Só acelera reto no eixo X se estiver REALMENTE atrás e alinhado com a bola na parede
            if -0.22 < along_x < -0.02 and lat_err < 0.03:
                self.dribbling = True
                err = wrap_angle(x_heading - me[2])
                w = clamp(KW * err, -W_MAX, W_MAX)
                if abs(err) > 0.20:
                    return 0.0, w
                return V_MAX, w

            self.dribbling = False
            prep_x = clamp(ball[0] - self.attack * 0.13, -FIELD_L / 2 + MARGIN, FIELD_L / 2 - MARGIN)
            # Se está do lado ou à frente da bola, recua abrindo por dentro do campo para não bater nela
            if along_x >= -0.02 and abs(me[1] - ball[1]) < 0.12:
                inside_y = clamp(ball[1] - wall_sign * 0.15, -WALL_STICK_Y, WALL_STICK_Y)
                v, w, _ = go_to(me, (prep_x, inside_y), d_slow=0.03, reverse=True)
                return v, w

            v, w, _ = go_to(me, (prep_x, lane_y), d_slow=0.03, reverse=True)
            return v, w

        ux, uy = self.push_dir(ball)
        rx, ry = me[0] - ball[0], me[1] - ball[1]
        dist = math.hypot(rx, ry)
        along = rx * ux + ry * uy
        lateral = -rx * uy + ry * ux
        err = wrap_angle(math.atan2(ball[1] - me[1], ball[0] - me[0]) - me[2])

        # 2. Mantém a condução contínua mesmo com pequenas batidas ou se a bola correr na frente
        if self.dribbling:
            if along < 0.025 and abs(lateral) < DRIBBLE_KEEP_LAT and dist < DRIBBLE_KEEP_DIST and abs(err) < DRIBBLE_KEEP_ERR:
                return self.push_cmd(me, ball, ux, uy)
            self.dribbling = False

        # 3. Inicia condução contínua um pouco antes (dist < 0.32) para não frear na aproximação
        if along < -0.015 and abs(lateral) < ALIGN_LAT and abs(err) < ALIGN_ERR and dist < 0.32:
            self.dribbling = True
            return self.push_cmd(me, ball, ux, uy)

        if along >= -0.015:
            v, w, _ = go_to(me, self.orbit_point(me, ball, ux, uy), d_slow=0.03, reverse=True)
            return v, w

        prep = in_field((ball[0] - PREP * ux, ball[1] - PREP * uy))
        if math.hypot(me[0] - prep[0], me[1] - prep[1]) < 0.04:
            if abs(err) < ALIGN_ERR:
                self.dribbling = True
                return self.push_cmd(me, ball, ux, uy)
            return 0.0, clamp(KW * err, -W_MAX, W_MAX)

        v, w, _ = go_to(me, prep, d_slow=0.03, reverse=True)
        return v, w

    def striker_safe(self, me, ball):
        now = rospy.get_time()
        if now < self.unstick_until:
            v, w, _ = go_to(me, ball, d_slow=1e-3)
            return v, w
        if self.safe_active:
            self.safe_active = False

        v, w = self.striker_cmd(me, ball)
        if abs(v) < 0.02 and abs(w) < 0.3:
            if self.idle_since is None:
                self.idle_since = now
            elif now - self.idle_since > IDLE_S:
                self.idle_since, self.unstick_until = None, now + 0.8
                self.safe_active = True
        else:
            self.idle_since = None
        return v, w

    # --- apoio ativo (ataque próximo, defesa em segunda linha e espera de reset) ------

    def support(self, me, ball, striker_pose):
        """Apoio: segunda linha na defesa, acompanha o ataque recuado na mesma lateral e antecipa resets."""
        depth = -self.attack * ball[0]     # positivo no nosso campo, negativo no ataque
        atk_depth = self.attack * ball[0]  # positivo no campo adversário

        # 1. Bola na linha de fundo/canto: espera recuado e na mesma lateral da bola
        if abs(ball[1]) >= ENDLINE_CORNER_Y:
            if atk_depth >= ENDLINE_STUCK_X:
                tgt = in_field((self.attack * RESET_WAIT_ATK_X,
                                clamp(ball[1] * 0.78, -RESET_WAIT_ATK_Y, RESET_WAIT_ATK_Y)))
                v, w, dist = go_to(me, tgt, v_max=V_MAX, d_slow=0.04, reverse=True)
                if dist < 0.04:
                    goal_x = self.attack * (FIELD_L / 2.0)
                    desired = math.atan2(0.0 - me[1], goal_x - me[0])
                    return 0.0, clamp(KW * wrap_angle(desired - me[2]), -W_MAX, W_MAX)
                return v, w
            if depth >= ENDLINE_STUCK_X:
                tgt = in_field((-self.attack * RESET_WAIT_DEF_X,
                                clamp(ball[1] * 0.65, -0.34, 0.34)))
                v, w, _ = go_to(me, tgt, v_max=V_MAX, d_slow=0.04, reverse=True)
                return v, w

        # 2. Defesa ativa: forma segunda linha atrás do atacante, protegendo o corredor do gol
        if depth > COVER_DEPTH:
            cover_depth = clamp(depth + 0.28, 0.25, 0.72)
            tgt_x = -self.attack * cover_depth
            tgt_y = clamp(ball[1] * 0.50, -0.32, 0.32)
            tgt = in_field((tgt_x, tgt_y))
            if math.hypot(tgt[0] - striker_pose[0], tgt[1] - striker_pose[1]) < 0.28:
                side = -1.0 if striker_pose[1] >= 0.0 else 1.0
                tgt = in_field((tgt_x, clamp(tgt_y + side * 0.24, -0.44, 0.44)))
        else:
            # 3. Ataque em jogo: fica recuado (18 cm atrás da bola) e próximo da mesma lateral
            inward = -1.0 if ball[1] >= 0.0 else 1.0
            target_x = ball[0] - self.attack * SUPPORT_BACK
            target_y = clamp(ball[1] + inward * SUPPORT_SIDE, -0.48, 0.48)
            tgt = in_field((target_x, target_y))
            if math.hypot(tgt[0] - striker_pose[0], tgt[1] - striker_pose[1]) < 0.15:
                tgt = in_field((target_x - self.attack * 0.10,
                                clamp(ball[1] + inward * (SUPPORT_SIDE + 0.06), -0.48, 0.48)))

        v, w, _ = go_to(me, tgt, v_max=V_MAX, d_slow=0.04, reverse=True)
        return v, w

    # --- ciclo principal --------------------------------------------------------------

    def compute_commands(self, poses, ball):
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
        cmds = [(0.0, 0.0)] * 3
        if self.ball is not None and len(self.poses) >= 6:
            try:
                out = list(self.compute_commands(dict(self.poses), self.ball))
                if len(out) == 3:
                    now = rospy.get_time()
                    cmds = []
                    for i, (v, w) in zip(self.ids, out):
                        if i == self.ids[0]:
                            self.stuck[i].reset()
                            self.stuck_active.discard(i)
                            rec = None
                        else:
                            rec = self.stuck[i].update(now, self.poses[i], v)
                        cmds.append(rec if rec is not None else (float(v), float(w)))
                else:
                    rospy.logwarn_throttle(2.0, "compute_commands devolveu %d comandos" % len(out))
            except Exception as exc:
                rospy.logwarn_throttle(2.0, "compute_commands falhou: %r" % (exc,))
                cmds = [(0.0, 0.0)] * 3
        self.log_status(rospy.get_time())
        for i, (v, w) in zip(self.ids, cmds):
            self.publish(i, v, w)

    def run(self):
        rate = rospy.Rate(RATE_HZ)
        for i in self.ids:
            self.publish(i, 0.0, 0.0)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()


if __name__ == "__main__":
    Soccer().run()