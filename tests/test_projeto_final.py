"""Testes offline dos três arquivos de jogo (sem ROS: rospy e as mensagens são simulados).

Garantem a robustez da Seção 6.0 do briefing:
  * o nó publica (0, 0) desde o primeiro ciclo, antes de receber qualquer mensagem;
  * uma exceção na lógica vira (0, 0) e o nó continua publicando;
  * só se publica no cmd_vel dos robôs do próprio time;
  * os comandos respeitam V_MAX e W_MAX.

Rode da raiz do repositório:  python tests/test_projeto_final.py
"""
import importlib.util
import os
import sys
import types
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _instala_psr_hero_utils():
    """Réplica de psr_hero.utils do kit (este repositório não traz o kit), para os testes rodarem em qualquer lugar."""
    import math

    utils = types.ModuleType("psr_hero.utils")
    utils.V_MAX, utils.W_MAX = 0.20, 4.0
    utils.clamp = lambda x, lo, hi: max(lo, min(hi, x))
    utils.wrap_angle = lambda a: (a + math.pi) % (2.0 * math.pi) - math.pi

    def yaw_from_quaternion(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    utils.yaw_from_quaternion = yaw_from_quaternion
    utils.pose_from_odom = lambda m: (m.pose.pose.position.x, m.pose.pose.position.y,
                                      yaw_from_quaternion(m.pose.pose.orientation))
    pkg = types.ModuleType("psr_hero")
    pkg.utils = utils
    sys.modules["psr_hero"], sys.modules["psr_hero.utils"] = pkg, utils


_instala_psr_hero_utils()


class Vec:
    x = y = z = 0.0


class Twist:
    def __init__(self):
        self.linear, self.angular = Vec(), Vec()


class FakeRospy(types.ModuleType):
    """Registra publicações; o laço roda N ciclos e termina."""

    def __init__(self):
        super().__init__("rospy")
        self.params, self.published, self.subs, self.cycles, self.limit = {}, [], [], 0, 5

    def reset(self, params, limit=5):
        self.params, self.published, self.subs, self.cycles, self.limit = params, [], [], 0, limit

    def init_node(self, *a, **k): pass
    def get_param(self, name, default=None): return self.params.get(name, default)
    def get_time(self): return self.cycles * 0.05
    def is_shutdown(self): return self.cycles >= self.limit
    def logwarn(self, *a): pass
    def logwarn_throttle(self, *a): pass
    def loginfo(self, *a): pass

    def Subscriber(self, topic, cls, cb, callback_args=None, queue_size=None):
        self.subs.append(topic)

    def Publisher(self, topic, cls, queue_size=None):
        outer = self

        class Pub:
            def publish(self, msg):
                outer.published.append((topic, msg.linear.x, msg.angular.z))
        return Pub()

    def Rate(self, hz):
        outer = self

        class R:
            def sleep(self): outer.cycles += 1
        return R()


rospy = FakeRospy()
sys.modules["rospy"] = rospy
for name, attrs in {
    "geometry_msgs": {}, "geometry_msgs.msg": {"Twist": Twist, "PointStamped": object},
    "nav_msgs": {}, "nav_msgs.msg": {"Odometry": object},
    "std_msgs": {}, "std_msgs.msg": {"Bool": object, "String": object},
}.items():
    m = types.ModuleType(name)
    m.__dict__.update(attrs)
    sys.modules[name] = m


def carregar(jogo):
    caminho = os.path.join(ROOT, "projeto-final", jogo + ".py")
    spec = importlib.util.spec_from_file_location("jogo_" + jogo, caminho)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class JogoBase:
    jogo = ""
    classe = ""
    params = {}
    topicos_proprios = set()

    def novo(self, limit=5):
        rospy.reset(dict(self.params), limit)
        return getattr(carregar(self.jogo), self.classe)()

    def test_publica_zero_no_primeiro_ciclo_sem_dados(self):
        no = self.novo()
        no.run()
        self.assertTrue(rospy.published, "não publicou nada")
        self.assertTrue(all(v == 0.0 and w == 0.0 for _, v, w in rospy.published))
        # publica ao menos uma vez por ciclo em cada robô controlado
        self.assertGreaterEqual(len(rospy.published), 5 * len(self.topicos_proprios))

    def test_so_publica_nos_proprios_robos(self):
        no = self.novo()
        no.run()
        self.assertEqual({t for t, _, _ in rospy.published}, self.topicos_proprios)

    def test_excecao_vira_zero_e_o_no_continua(self):
        no = self.novo(limit=6)
        self.dar_dados(no)
        self.forcar_excecao(no)
        no.run()  # não pode propagar a exceção
        self.assertGreaterEqual(len(rospy.published), 6 * len(self.topicos_proprios))
        self.assertTrue(all(v == 0.0 and w == 0.0 for _, v, w in rospy.published))

    def test_comandos_sao_saturados(self):
        no = self.novo(limit=3)
        self.dar_dados(no)
        self.forcar_comando(no, 9.0, -99.0)
        no.run()
        v_max, w_max = no_utils()
        for _, v, w in rospy.published:
            self.assertLessEqual(abs(v), v_max + 1e-12)
            self.assertLessEqual(abs(w), w_max + 1e-12)
        self.assertTrue(any(abs(v) > 0 for _, v, _ in rospy.published), "o comando forçado não saiu")


def no_utils():
    from psr_hero.utils import V_MAX, W_MAX
    return V_MAX, W_MAX


class TestSumo(JogoBase, unittest.TestCase):
    jogo, classe = "sumo", "Sumo"
    params = {"~robot": 1, "~opponent": 0}
    topicos_proprios = {"/hero_1/cmd_vel"}

    def dar_dados(self, no):
        no.me, no.opp = (0.1, 0.0, 3.14), (-0.1, 0.0, 0.0)

    def forcar_excecao(self, no):
        no.compute_command = lambda *a: 1 / 0

    def forcar_comando(self, no, v, w):
        no.compute_command = lambda *a: (v, w)


class TestSoccer(JogoBase, unittest.TestCase):
    jogo, classe = "soccer", "Soccer"
    params = {"~team": "red"}
    topicos_proprios = {"/hero_3/cmd_vel", "/hero_4/cmd_vel", "/hero_5/cmd_vel"}

    def dar_dados(self, no):
        no.poses = {i: (0.0, 0.0, 0.0) for i in range(6)}
        no.ball = (0.0, 0.0)

    def forcar_excecao(self, no):
        no.compute_commands = lambda *a: 1 / 0

    def forcar_comando(self, no, v, w):
        no.compute_commands = lambda *a: [(v, w)] * 3

    def test_lado_azul_e_vermelho(self):
        for team, ids, attack in (("blue", [0, 1, 2], +1), ("red", [3, 4, 5], -1)):
            rospy.reset({"~team": team})
            no = carregar("soccer").Soccer()
            self.assertEqual(no.ids, ids)
            self.assertEqual(no.attack, attack)

    def test_numero_errado_de_comandos_nao_derruba(self):
        no = self.novo(limit=3)
        self.dar_dados(no)
        no.compute_commands = lambda *a: [(0.1, 0.0)]  # só um
        no.run()
        self.assertTrue(all(v == 0.0 and w == 0.0 for _, v, w in rospy.published))


class TestCtf(JogoBase, unittest.TestCase):
    jogo, classe = "ctf", "CaptureTheFlag"
    params = {"~robot": 2, "~team": "red", "~role": "defender", "~opponents": [0, 1]}
    topicos_proprios = {"/hero_2/cmd_vel"}

    def dar_dados(self, no):
        no.me = (0.5, 0.0, 3.14)
        no.opp = {0: (-0.5, 0.0, 0.0), 1: (-0.5, 0.3, 0.0)}
        no.enemy_flag, no.own_flag = (-0.62, 0.0), (0.62, 0.0)

    def forcar_excecao(self, no):
        no.compute_command = lambda *a: 1 / 0

    def forcar_comando(self, no, v, w):
        no.compute_command = lambda *a: (v, w)

    def test_assina_as_bandeiras_certas_para_cada_lado(self):
        rospy.reset({"~robot": 2, "~team": "red", "~opponents": [0, 1]})
        carregar("ctf").CaptureTheFlag()
        self.assertIn("/flag/blue", rospy.subs)   # vermelho rouba a bandeira azul
        self.assertIn("/flag/red", rospy.subs)
        self.assertIn("/hero_2/has_flag", rospy.subs)
        self.assertNotIn("/hero_0/has_flag", rospy.subs)

    def test_modo_1x1_sem_colega(self):
        rospy.reset({"~robot": 0, "~team": "blue", "~role": "attacker", "~opponents": [1]})
        no = carregar("ctf").CaptureTheFlag()
        no.me, no.enemy_flag, no.own_flag = (0.0, 0.0, 0.0), (0.62, 0.0), (-0.62, 0.0)
        no.run()  # sem nenhum oponente na odometria ainda: não pode quebrar
        self.assertTrue(rospy.published)


if __name__ == "__main__":
    unittest.main(verbosity=2)
