"""Base dos experimentos de física da Fase 1 (E1 a E3). FERRAMENTA LOCAL: não é um arquivo de jogo.

Cada exp_*.py herda de ExpNode: assina a odometria dos dois robôs e o /game/state, controla o hero_<~robot>
e grava um CSV (a cada ciclo de 20 Hz) em /ws/psr_exercicios/registros/fase1/ (a única pasta gravável do
container). Ao terminar, imprime EXPERIMENT_DONE e fica parado, publicando (0, 0).

Colunas do CSV: wall, rnd, rt, x0, y0, yaw0, x1, y1, yaw1, v, w, tag
  rnd = rodada atual (vitórias + empates do /game/state); rt = round_time; v, w = comando publicado.
"""
import json
import math
import os
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from psr_hero.utils import V_MAX, W_MAX, clamp, pose_from_odom, wrap_angle

RING_RADIUS = 0.40
RATE_HZ = 20.0
OUT_DIR = "/ws/psr_exercicios/registros/fase1"


class ExpNode:
    name = "exp"

    def __init__(self):
        rospy.init_node(self.name)
        self.me = rospy.get_param("~robot", 0)
        self.opp = rospy.get_param("~opponent", 1)
        self.pose = {0: None, 1: None}
        self.rt = None
        self.rnd = 0
        self.tag = "init"
        self.pub = rospy.Publisher("/hero_%d/cmd_vel" % self.me, Twist, queue_size=1)
        for i in (0, 1):
            rospy.Subscriber("/hero_%d/odom" % i, Odometry, self.on_odom, callback_args=i, queue_size=1)
        rospy.Subscriber("/game/state", String, self.on_state, queue_size=1)
        try:
            os.makedirs(OUT_DIR)
        except OSError:
            pass
        path = os.path.join(OUT_DIR, "%s.csv" % self.csv_name())
        self.f = open(path, "w")
        self.f.write("wall,rnd,rt,x0,y0,yaw0,x1,y1,yaw1,v,w,tag\n")
        self.done = False
        rospy.loginfo("%s: gravando em %s" % (self.name, path))

    def csv_name(self):
        return self.name

    def on_odom(self, msg, i):
        self.pose[i] = pose_from_odom(msg)

    def on_state(self, msg):
        try:
            g = json.loads(msg.data)
        except ValueError:
            return
        self.rt = g.get("round_time")
        self.rnd = int(sum(g.get("wins", [0, 0])) + g.get("draws", 0))

    def publish(self, v, w):
        cmd = Twist()
        cmd.linear.x = clamp(v, -V_MAX, V_MAX)
        cmd.angular.z = clamp(w, -W_MAX, W_MAX)
        self.pub.publish(cmd)
        return cmd.linear.x, cmd.angular.z

    def log(self, v, w):
        a, b = self.pose[0], self.pose[1]
        if a is None or b is None:
            return
        self.f.write("%.3f,%d,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%s\n" % (
            time.time(), self.rnd, "" if self.rt is None else "%.2f" % self.rt,
            a[0], a[1], a[2], b[0], b[1], b[2], v, w, self.tag))

    def step(self):
        """Devolve (v, w). Subclasses definem self.tag e controlam self.done."""
        return 0.0, 0.0

    def finish(self):
        self.done = True
        self.tag = "done"
        self.f.flush()
        rospy.loginfo("EXPERIMENT_DONE %s" % self.name)

    def run(self):
        rate = rospy.Rate(RATE_HZ)
        self.publish(0.0, 0.0)
        while not rospy.is_shutdown():
            v, w = 0.0, 0.0
            if self.pose[0] is not None and self.pose[1] is not None and self.rt is not None and not self.done:
                try:
                    v, w = self.step()
                except Exception as exc:
                    rospy.logwarn_throttle(2.0, "erro no experimento: %r" % (exc,))
                    v, w = 0.0, 0.0
            v, w = self.publish(v, w)
            self.log(v, w)
            self.f.flush()
            rate.sleep()


def aim(me, target, v_max=V_MAX, kw=5.0, stop_dist=0.0):
    """Controle 'ir até um ponto': devolve (v, w, dist, erro angular)."""
    dx, dy = target[0] - me[0], target[1] - me[1]
    dist = math.hypot(dx, dy)
    err = wrap_angle(math.atan2(dy, dx) - me[2])
    w = clamp(kw * err, -W_MAX, W_MAX)
    v = v_max * max(0.0, math.cos(err)) if dist > stop_dist else 0.0
    return v, w, dist, err
