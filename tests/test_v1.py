"""Testes offline da lógica da v1 (Fase 2): controlador base, sumô, rouba-bandeira e futebol.

Reaproveita os stubs de rospy e das mensagens de test_projeto_final.py. Rode da raiz do repositório:
    python tests/test_v1.py
"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_projeto_final import carregar, rospy  # noqa: E402


class Msg:
    def __init__(self, data):
        self.data = data


class TestControladorBase(unittest.TestCase):
    def setUp(self):
        rospy.reset({})
        self.go_to = carregar("sumo").go_to

    def test_vai_para_frente_quando_o_alvo_esta_a_frente(self):
        v, w, d = self.go_to((0, 0, 0.0), (0.3, 0.0))
        self.assertGreater(v, 0.19)
        self.assertAlmostEqual(w, 0.0)
        self.assertAlmostEqual(d, 0.3)

    def test_gira_para_o_lado_certo(self):
        self.assertGreater(self.go_to((0, 0, 0.0), (0.0, 0.3))[1], 0.0)  # alvo à esquerda
        self.assertLess(self.go_to((0, 0, 0.0), (0.0, -0.3))[1], 0.0)

    def test_re_so_quando_permitida(self):
        self.assertLess(self.go_to((0, 0, 0.0), (-0.3, 0.0), reverse=True)[0], 0.0)
        v, w, _ = self.go_to((0, 0, 0.0), (-0.3, 0.0), reverse=False)
        self.assertEqual(v, 0.0)
        self.assertNotEqual(w, 0.0)

    def test_alvo_no_proprio_robo_nao_divide_por_zero(self):
        self.assertEqual(self.go_to((0.1, 0.1, 1.0), (0.1, 0.1)), (0.0, 0.0, 0.0))

    def test_saturacao(self):
        for pose in [(0, 0, 0.0), (0, 0, 3.0), (0, 0, -2.5)]:
            v, w, _ = self.go_to(pose, (5.0, 5.0), reverse=True)
            self.assertLessEqual(abs(v), 0.2 + 1e-9)
            self.assertLessEqual(abs(w), 4.0 + 1e-9)


class TestSumoV1(unittest.TestCase):
    def novo(self):
        rospy.reset({"~robot": 0, "~opponent": 1})
        return carregar("sumo").Sumo()

    def test_engaja_de_frente(self):
        no = self.novo()
        v, _ = no.compute_command((-0.18, 0, 0.0), (0.18, 0, math.pi), 0.0, 0.0)
        self.assertGreater(v, 0.19)
        self.assertEqual(no.estado, "ENGAJAR")

    def test_borda_tem_prioridade_e_volta_de_re(self):
        no = self.novo()
        v, _ = no.compute_command((0.35, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0, 0.0)
        self.assertEqual(no.estado, "BORDA")
        self.assertLess(v, 0.0)  # ré para o centro, sem virar as costas ao adversário

    def test_histerese_da_borda(self):
        no = self.novo()
        no.compute_command((0.33, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0, 0.0)
        no.compute_command((0.30, 0.0, math.pi), (0.0, 0.0, 0.0), 0.0, 0.0)
        self.assertEqual(no.estado, "BORDA")  # abaixo de R_SAFE mas ainda acima de R_SAFE - histerese
        no.compute_command((0.20, 0.0, math.pi), (0.0, 0.0, 0.0), 0.0, 0.0)
        self.assertEqual(no.estado, "ENGAJAR")

    def test_reinicio_de_rodada_zera_o_estado(self):
        no = self.novo()
        no.compute_command((0.35, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0, 0.0)
        no.on_state(Msg('{"round_time": 30.0}'))
        no.on_state(Msg('{"round_time": 0.4}'))
        self.assertEqual(no.estado, "ENGAJAR")

    def test_estado_invalido_nao_quebra(self):
        no = self.novo()
        no.on_state(Msg("isto nao e json"))
        no.on_state(Msg('{"sem_round_time": 1}'))
        no.on_state(Msg("[1, 2]"))


class TestStuck(unittest.TestCase):
    def test_detecta_travamento_e_destrava(self):
        s = carregar("ctf").Stuck()
        pos, t, rec = (0.0, 0.0), 0.0, None
        while t < 3.0 and rec is None:
            rec = s.update(t, pos, 0.2)  # comando alto, robô parado
            t += 0.05
        self.assertIsNotNone(rec)
        self.assertLess(rec[0], 0.0)  # ré
        self.assertGreater(t, 1.4)  # só depois de ~1,5 s

    def test_nao_dispara_se_o_robo_se_mexe_ou_nao_manda_andar(self):
        s = carregar("ctf").Stuck()
        for k in range(100):
            self.assertIsNone(s.update(k * 0.05, (k * 0.01, 0.0), 0.2))  # andando de verdade
        s.reset()
        for k in range(100):
            self.assertIsNone(s.update(k * 0.05, (0.0, 0.0), 0.0))  # parado, sem mandar andar


class TestCtfV1(unittest.TestCase):
    def novo(self, **params):
        base = {"~robot": 0, "~team": "blue", "~role": "attacker", "~opponents": [2, 3]}
        base.update(params)
        rospy.reset(base)
        return carregar("ctf").CaptureTheFlag()

    def test_atacante_vai_para_a_bandeira_inimiga(self):
        no = self.novo()
        v, _ = no.compute_command((-0.3, 0.0, 0.0), [(0.4, 0.5, 0.0)], (0.62, 0.0), (-0.62, 0.0), False, 0.0)
        self.assertGreater(v, 0.1)
        self.assertEqual(no.estado, "IR_BANDEIRA")

    def test_atacante_com_a_bandeira_volta_para_a_base(self):
        no = self.novo()
        no.compute_command((0.3, 0.0, 0.0), [], (0.62, 0.0), (-0.62, 0.0), True, 0.0)
        self.assertEqual(no.estado, "VOLTAR")

    def test_atacante_desvia_do_defensor(self):
        no = self.novo()
        _, w = no.compute_command((0.30, 0.0, 0.0), [(0.45, 0.02, 3.14)], (0.62, 0.0), (-0.62, 0.0), False, 0.0)
        self.assertNotAlmostEqual(w, 0.0, places=2)  # defensor no caminho: não segue em linha reta

    def test_defensor_planta_sobre_a_bandeira_e_encara_o_intruso(self):
        no = self.novo(**{"~role": "defender", "~robot": 1})
        home = no.home
        v, w = no.compute_command((home[0], home[1], 0.0), [(-0.3, 0.3, 0.0)], (0.62, 0.0), home, False, 0.0)
        self.assertEqual(v, 0.0)
        self.assertGreater(w, 0.0)  # intruso à esquerda e acima
        self.assertEqual(no.estado, "PLANTADO")

    def test_defensor_volta_para_a_bandeira_se_deslocado(self):
        no = self.novo(**{"~role": "defender", "~robot": 1})
        v, _ = no.compute_command((-0.50, 0.0, 0.0), [], (0.62, 0.0), no.home, False, 0.0)
        self.assertNotEqual(v, 0.0)

    def test_defensor_persegue_portador_na_propria_metade(self):
        no = self.novo(**{"~role": "defender", "~robot": 1})
        flag = (-0.40, 0.10)  # a nossa bandeira foi levada
        no.compute_command((-0.62, 0.0, 0.0), [(-0.40, 0.10, 0.0)], (0.62, 0.0), flag, False, 0.0)
        self.assertEqual(no.estado, "PERSEGUIR")
        no.compute_command((-0.62, 0.0, 0.0), [(0.10, 0.10, 0.0)], (0.62, 0.0), flag, False, 0.0)  # cruzou x = 0
        self.assertEqual(no.estado, "PLANTADO")

    def test_lado_vermelho_espelha(self):
        no = self.novo(**{"~team": "red", "~robot": 2, "~opponents": [0, 1]})
        self.assertEqual(no.home, (0.62, 0.0))
        self.assertEqual(no.side, 1.0)

    def test_sem_oponentes_nao_quebra(self):
        for role in ("attacker", "defender"):
            no = self.novo(**{"~role": role})
            no.compute_command((0.0, 0.0, 0.0), [], (0.62, 0.0), (-0.62, 0.0), False, 0.0)


class TestSoccerV1(unittest.TestCase):
    def novo(self, team="blue"):
        rospy.reset({"~team": team})
        return carregar("soccer").Soccer()

    def poses(self, over=None):
        base = {0: (-0.78, 0.0, 0.0), 1: (-0.35, 0.25, 0.0), 2: (-0.35, -0.25, 0.0),
                3: (0.78, 0.0, math.pi), 4: (0.35, 0.25, math.pi), 5: (0.35, -0.25, math.pi)}
        base.update(over or {})
        return base

    def test_goleiro_segue_a_bola_na_linha_do_gol(self):
        no = self.novo("blue")
        cmds = no.compute_commands(self.poses(), (0.0, 0.5))
        self.assertEqual(no.roles[0], "GOLEIRO")
        self.assertNotEqual(cmds[0][1], 0.0)

    def test_papeis_do_time_vermelho(self):
        no = self.novo("red")
        no.compute_commands(self.poses(), (0.0, 0.0))
        self.assertEqual(no.roles[3], "GOLEIRO")
        self.assertEqual(sorted(no.roles.values()), ["APOIO", "ATACANTE", "GOLEIRO"])

    def test_atacante_e_o_mais_perto_da_bola(self):
        no = self.novo("blue")
        no.compute_commands(self.poses(), (-0.3, 0.2))
        self.assertEqual(no.striker, 1)
        self.assertEqual(no.roles[2], "APOIO")

    def test_histerese_do_atacante(self):
        no = self.novo("blue")
        no.compute_commands(self.poses(), (-0.3, 0.2))
        self.assertEqual(no.striker, 1)
        # o robô 2 fica ~11% mais rápido (ETA 0,49 s contra 0,55 s): abaixo dos 20%, não troca
        no.compute_commands(self.poses({2: (-0.35, 0.24, 0.0)}), (-0.3, 0.2))
        self.assertEqual(no.striker, 1)
        # o robô 2 fica ~34% mais rápido (ETA 0,37 s): passa dos 20%, troca
        no.compute_commands(self.poses({2: (-0.35, 0.22, 0.0)}), (-0.3, 0.2))
        self.assertEqual(no.striker, 2)

    def test_atras_da_bola_avanca_para_chutar(self):
        no = self.novo("blue")
        v, w = no.striker_cmd((-0.10, 0.0, 0.0), (0.0, 0.0))
        self.assertGreater(v, 0.15)
        self.assertAlmostEqual(w, 0.0, places=1)

    def test_na_frente_da_bola_contorna(self):
        no = self.novo("blue")
        _, w = no.striker_cmd((0.20, 0.0, math.pi), (0.0, 0.0))  # mais perto do gol adversário que a bola
        self.assertNotEqual(w, 0.0)

    def test_bola_em_cima_do_robo_nao_divide_por_zero(self):
        no = self.novo("blue")
        no.compute_commands(self.poses({1: (0.0, 0.0, 0.0)}), (0.0, 0.0))
        no.compute_commands(self.poses(), (0.9, 0.0))

    def test_gol_reinicia_o_atacante(self):
        no = self.novo("blue")
        no.striker = 1
        no.on_state(Msg('{"blue": 0, "red": 0}'))
        no.on_state(Msg('{"blue": 1, "red": 0}'))
        self.assertIsNone(no.striker)

    def test_regressao_bola_colada_no_proprio_fundo_nao_deixa_o_atacante_parado(self):
        # situação do simulador (vermelho): bola em (0,89; 0,02) e o atacante exatamente no ponto de contorno
        no = self.novo("red")
        poses = self.poses({3: (0.80, 0.02, 2.0), 4: (0.35, 0.0, 2.5), 5: (0.85, 0.18, 1.1)})
        cmds = no.compute_commands(poses, (0.89, 0.02))
        v, w = cmds[no.ids.index(no.striker)]
        self.assertGreater(abs(v) + abs(w), 0.05)

    def test_goleiro_limpa_a_bola_funda_na_propria_area(self):
        no = self.novo("red")  # o gol do vermelho fica em +x; a bola está atrás do goleiro, colada no fundo
        poses = self.poses({3: (0.80, 0.02, math.pi), 4: (0.35, 0.0, 2.5), 5: (0.30, 0.20, 2.5)})
        no.compute_commands(poses, (0.89, 0.02))
        self.assertEqual(no.striker, 3)  # o goleiro é o mais perto: ele limpa
        self.assertEqual(no.roles[3], "ATACANTE")

    def test_goleiro_continua_goleiro_com_a_bola_longe_da_area(self):
        no = self.novo("red")
        no.compute_commands(self.poses(), (0.2, 0.0))
        self.assertEqual(no.roles[3], "GOLEIRO")
        self.assertNotEqual(no.striker, 3)

    def test_direcao_de_empurro_limpa_para_a_lateral_perto_do_proprio_fundo(self):
        no = self.novo("red")  # ataca -x: o gol dela fica em +x
        ux, uy = no.push_dir((0.89, 0.02))
        self.assertAlmostEqual(abs(uy), 0.894, places=2)  # sobe/desce ao longo da linha de fundo
        self.assertLess(ux, 0.0)  # e um pouco para frente (para o campo)

    def test_direcao_de_empurro_normal_vai_para_o_gol_adversario(self):
        for team, sinal in (("blue", 1.0), ("red", -1.0)):
            ux, uy = self.novo(team).push_dir((0.0, 0.0))
            self.assertAlmostEqual(ux, sinal)
            self.assertAlmostEqual(uy, 0.0)

    def test_o_atacante_nunca_fica_parado_em_malha_fechada(self):
        """Propriedade: com a bola fixa em qualquer lugar, o comando do atacante não fica em (0, 0) por > 1 s."""
        import random
        rng = random.Random(7)
        piores = 0
        for team in ("blue", "red"):
            no = self.novo(team)
            for _ in range(150):
                ball = (rng.uniform(-0.88, 0.88), rng.uniform(-0.58, 0.58))
                me = [rng.uniform(-0.85, 0.85), rng.uniform(-0.55, 0.55), rng.uniform(-math.pi, math.pi)]
                zeros = 0
                for _ in range(200):  # 10 s a 20 Hz, cinemática simples; a bola não se move
                    v, w = no.striker_cmd(tuple(me), ball)
                    zeros = zeros + 1 if (abs(v) < 0.01 and abs(w) < 0.01) else 0
                    piores = max(piores, zeros)
                    me[2] += w * 0.05
                    me[0] = max(-0.86, min(0.86, me[0] + v * math.cos(me[2]) * 0.05))
                    me[1] = max(-0.56, min(0.56, me[1] + v * math.sin(me[2]) * 0.05))
        self.assertLess(piores, 20, "o atacante ficou %d ciclos seguidos parado" % piores)

    def test_alvos_ficam_dentro_do_campo(self):
        x, y = carregar("soccer").in_field((5.0, -5.0))
        self.assertLessEqual(abs(x), 0.9)
        self.assertLessEqual(abs(y), 0.6)


if __name__ == "__main__":
    unittest.main(verbosity=2)
