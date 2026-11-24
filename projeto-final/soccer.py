#!/usr/bin/env python3
"""Futebol (RTech Tabajara): um processo controla os três robôs do time. Parâmetro: ~team.

`blue` (robôs 0, 1, 2) ataca +x; `red` (robôs 3, 4, 5) ataca -x. Campo 1,8 x 1,2 m, centro (0, 0), gols de
0,40 m no meio dos lados curtos. O índice 0 de self.ids é o goleiro (ordem de nascimento).

v4. Papéis:
  GOLEIRO   defende a linha e sai para rebater/tirar a bola sempre que ela entra na área.
  ATACANTE  escolhido dinamicamente entre os dois jogadores de linha por ETA + histerese; conduz a bola
            continuamente até o gol sem frear durante o drible.
  APOIO     participa ativamente: no ataque fecha o segundo pau / rebote pronto para assumir a bola;
            na defesa faz marcação/cobertura ativa entre a bola e o gol.
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
KEEPER_HOME_X = 0.84
KEEPER_SWEEP_X = 0.56
KEEPER_Y = 0.16
KEEPER_TRACK_DEPTH = 0.10
KEEPER_PREDICT_TIME = 2.5
KEEPER_CLEAR_DEPTH = 0.52     # bola mais funda que isso no nosso campo -> goleiro sai para tirar
KEEPER_CLEAR_EXIT = 0.42      # histerese para o goleiro voltar para a meta
KEEPER_CLEAR_Y = 0.36         # largura da área onde o goleiro sai na bola
KEEPER_MAX_OUT_X = 0.45       # limite máximo que o goleiro avança para não abandonar o gol
GK_VMIN = 0.03

# Atacante e condução contínua
PREP = 0.11
ALIGN_ERR = 0.42              # rad (~24 graus) para iniciar condução
ALIGN_LAT = 0.07              # m de desvio lateral para iniciar condução
DRIBBLE_KEEP_DIST = 0.24      # m, distância máxima para manter modo condução contínua
DRIBBLE_KEEP_LAT = 0.12       # m, tolerância lateral mais folgada enquanto já está conduzindo
DRIBBLE_KEEP_ERR = 0.70       # rad, tolerância angular enquanto já está conduzindo
SIDELINE_ZONE = 0.12
R_ORB = 0.15
ORB_STEP = 0.85

# Troca dinâmica de atacante (histerese)
SWITCH_ETA_MARGIN = 0.30      # s de vantagem mínima para o apoio virar atacante
SWITCH_HOLD_TIME = 0.25       # s sustentando a vantagem antes de trocar

# Apoio ativo (ataque e defesa)
SUPPORT_SIDE = 0.24
SUPPORT_DEF_MIN = 0.15
SUPPORT_DEF_MAX = 0.44
SUPPORT_DEF_Y = 0.28
COVER_DEPTH = 0.20

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

        # Se o atacante atual já está conduzindo a bola, exige uma vantagem ainda maior para trocar
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

    # --- goleiro (defesa + saída para tirar a bola da área) ---------------------------

    def keeper_target(self, ball, now=None):
        now = rospy.get_time() if now is None else now
        depth = -self.attack * ball[0]
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
                target_y = ball[1] * 0.65
        elif depth > KEEPER_TRACK_DEPTH:
            target_y = ball[1] * 0.65
        target_y = clamp(target_y, -KEEPER_Y, KEEPER_Y)

        dt = clamp(now - self.keeper_filter_time, 0.0, 0.15)
        alpha = clamp(dt * (12.0 if intercepting else 5.0), 0.0, 1.0) if dt > 0.0 else 1.0
        self.keeper_x_filtered += alpha * (target_x_abs - self.keeper_x_filtered)
        self.keeper_y_filtered += alpha * (target_y - self.keeper_y_filtered)
        self.keeper_filter_time = now
        return -self.attack * self.keeper_x_filtered, clamp(self.keeper_y_filtered, -KEEPER_Y, KEEPER_Y)

    def keeper(self, me, ball):
        """Goleiro móvel: sai da área para rebater a bola quando ela entra na zona de perigo."""
        depth = -self.attack * ball[0]
        me_depth = -self.attack * me[0]
        ball_in_box = depth > KEEPER_CLEAR_DEPTH and abs(ball[1]) < KEEPER_CLEAR_Y
        ball_still_danger = depth > KEEPER_CLEAR_EXIT and abs(ball[1]) < KEEPER_CLEAR_Y + 0.08

        # Ativa/mantém modo de saída para tirar a bola da área
        if ball_in_box and me_depth > KEEPER_MAX_OUT_X:
            self.keeper_clearing = True
        elif not ball_still_danger or me_depth <= KEEPER_MAX_OUT_X:
            self.keeper_clearing = False

        if self.keeper_clearing:
            # Só avança direto se estiver atrás da bola (para não fazer gol contra)
            behind_ball = self.attack * (me[0] - ball[0]) < -0.02
            if behind_ball:
                # Empurra a bola para frente e levemente para fora da área
                clear_y = math.copysign(0.35, ball[1]) if abs(ball[1]) > 0.05 else 0.0
                ux = float(self.attack)
                uy = clear_y - ball[1]
                n = math.hypot(ux, uy)
                return self.push_cmd(me, ball, ux / n, uy / n)
            else:
                # Se a bola passou do goleiro, volta rápido para trás dela na linha do gol
                retreat = in_field((-self.attack * KEEPER_HOME_X, clamp(ball[1], -KEEPER_Y, KEEPER_Y)))
                v, w, _ = go_to(me, retreat, v_max=V_MAX, kw=5.0, d_slow=0.04, reverse=True)
                return v, w

        target = self.keeper_target(ball)
        v, w, _ = go_to(me, target, v_max=V_MAX * 0.90, kw=4.2, d_slow=0.06, reverse=True)
        return v, w

    # --- ataque e condução contínua ---------------------------------------------------

    def goal_target(self, ball):
        limit = GOAL_W / 2.0 - GOAL_MARGIN
        return (self.attack * (FIELD_L / 2.0), clamp(GOAL_FOLLOW * ball[1], -limit, limit))

    def push_dir(self, ball):
        depth = -self.attack * ball[0]
        if depth >= 0.15 or abs(ball[1]) >= FIELD_W / 2.0 - SIDELINE_ZONE:
            ux, uy = float(self.attack), 0.0
        else:
            goal_x, goal_y = self.goal_target(ball)
            ux, uy = goal_x - ball[0], goal_y - ball[1]
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
        # Aponta para um ponto ligeiramente à frente da bola na direção do gol
        aim_x = ball[0] + 0.08 * ux
        aim_y = ball[1] + 0.08 * uy
        desired = math.atan2(aim_y - me[1], aim_x - me[0])
        err = wrap_angle(desired - me[2])
        w = clamp(KW * err, -W_MAX, W_MAX)
        # Mantém tração contínua para nunca perder o contato com a bola
        if abs(err) < 0.90:
            v = V_MAX * max(0.65, math.cos(err))
        else:
            v = V_MAX * max(0.0, math.cos(err))
        return v, w

    def striker_cmd(self, me, ball):
        ux, uy = self.push_dir(ball)
        rx, ry = me[0] - ball[0], me[1] - ball[1]
        dist = math.hypot(rx, ry)
        along = rx * ux + ry * uy      # negativo quando estamos atrás da bola
        lateral = -rx * uy + ry * ux   # desvio lateral em relação à linha de ataque
        err = wrap_angle(math.atan2(ball[1] - me[1], ball[0] - me[0]) - me[2])

        # 1. Histerese de condução: se já está conduzindo, só larga se perder a bola de verdade
        if self.dribbling:
            if along < 0.01 and abs(lateral) < DRIBBLE_KEEP_LAT and dist < DRIBBLE_KEEP_DIST and abs(err) < DRIBBLE_KEEP_ERR:
                return self.push_cmd(me, ball, ux, uy)
            self.dribbling = False

        # 2. Inicia condução contínua assim que estiver atrás e alinhado com a bola
        if along < -0.015 and abs(lateral) < ALIGN_LAT and abs(err) < ALIGN_ERR and dist < 0.24:
            self.dribbling = True
            return self.push_cmd(me, ball, ux, uy)

        # 3. Se está do lado errado da bola, contorna por órbita (exceto colado na lateral)
        if along >= -0.015:
            if abs(ball[1]) >= FIELD_W / 2.0 - SIDELINE_ZONE:
                prep = in_field((ball[0] - self.attack * PREP, ball[1]))
                v, w, _ = go_to(me, prep, d_slow=0.04, reverse=True)
                return v, w
            v, w, _ = go_to(me, self.orbit_point(me, ball, ux, uy), d_slow=0.03, reverse=True)
            return v, w

        # 4. Aproximação ao ponto de preparação atrás da bola
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

    # --- apoio ativo (ataque e defesa) ------------------------------------------------

    def support(self, me, ball, striker_pose):
        """Apoio dinâmico: participa do ataque (rebote/segundo pau) e da defesa (cobertura ativa)."""
        depth = -self.attack * ball[0]  # positivo no nosso campo, negativo no ataque

        if depth > COVER_DEPTH:
            # DEFESA ATIVA: posiciona-se na linha entre a bola e o nosso gol para interceptar rebotes
            # ou bloquear o avanço adversário sem atrapalhar o goleiro.
            cover_depth = clamp(depth - 0.18, SUPPORT_DEF_MIN, SUPPORT_DEF_MAX)
            tgt_x = -self.attack * cover_depth
            tgt_y = clamp(ball[1] * 0.65, -SUPPORT_DEF_Y, SUPPORT_DEF_Y)
            tgt = in_field((tgt_x, tgt_y))
            # Evita colidir com o atacante se ele estiver recuando na mesma linha
            if math.hypot(tgt[0] - striker_pose[0], tgt[1] - striker_pose[1]) < 0.20:
                side = -1.0 if striker_pose[1] >= 0.0 else 1.0
                tgt = in_field((tgt_x, clamp(tgt_y + side * 0.22, -0.45, 0.45)))
        else:
            # ATAQUE ATIVO: acompanha a jogada pronto para rebote ou finalizar no segundo pau.
            # Fica do lado oposto ao da bola/atacante, ligeiramente à frente no terço final.
            side = -1.0 if ball[1] >= 0.0 else 1.0
            in_final_third = (self.attack * ball[0]) > 0.30
            if in_final_third:
                # Fecha o segundo pau na boca da área adversária para empurrar qualquer rebote
                target_x = self.attack * 0.62
                target_y = side * 0.18
            else:
                # Acompanha em diagonal próximo à bola para roubar/assumir rápido se o atacante perder
                target_x = ball[0] + self.attack * 0.12
                target_y = clamp(ball[1] + side * SUPPORT_SIDE, -0.44, 0.44)

            tgt = in_field((target_x, target_y))
            if math.hypot(tgt[0] - striker_pose[0], tgt[1] - striker_pose[1]) < 0.22:
                tgt = in_field((target_x - self.attack * 0.12, clamp(ball[1] + side * 0.32, -0.46, 0.46)))

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