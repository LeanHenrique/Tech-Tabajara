"""Testes offline do logger (parte pura) e do analisador de registros.

Rode da raiz do repositório:  python tests/test_tools.py
"""
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "exercicios" / "tools"))

import log_game  # noqa: E402
import summarize  # noqa: E402

SUMO = {"wins": [0, 0], "draws": 0, "ring_radius": 0.4, "round_time": 5.0}
CTF = {"score": [0, 0], "tags": [0, 0], "held": [False] * 4, "carrying": [False] * 4, "round_time": 3.0}
SOCCER = {"blue": 0, "red": 0, "time": 10.0, "field": {}}


class LoggerTests(unittest.TestCase):
    def test_detectar_jogo(self):
        self.assertEqual(log_game.detectar_jogo(SUMO), "sumo")
        self.assertEqual(log_game.detectar_jogo(CTF), "ctf")
        self.assertEqual(log_game.detectar_jogo(SOCCER), "soccer")
        self.assertIsNone(log_game.detectar_jogo({"x": 1}))

    def test_placar(self):
        self.assertEqual(log_game.placar("sumo", {"wins": [2, 1]}), (2, 1))
        self.assertEqual(log_game.placar("ctf", {"score": [1, 3]}), (1, 3))
        self.assertEqual(log_game.placar("soccer", {"blue": 2, "red": 0}), (2, 0))

    def test_eventos_sumo(self):
        ev = log_game.eventos("sumo", SUMO, dict(SUMO, wins=[1, 0]))
        self.assertEqual(len(ev), 1)
        self.assertIn("hero_0", ev[0])
        ev = log_game.eventos("sumo", SUMO, dict(SUMO, draws=1))
        self.assertIn("empate", ev[0])

    def test_eventos_ctf_e_futebol(self):
        self.assertIn("azul", log_game.eventos("ctf", CTF, dict(CTF, score=[1, 0]))[0])
        self.assertIn("vermelho", log_game.eventos("soccer", SOCCER, dict(SOCCER, red=1))[0])
        self.assertIn("hold", log_game.eventos("ctf", CTF, dict(CTF, tags=[1, 0]))[0])

    def test_reinicio_de_rodada(self):
        ev = log_game.eventos("sumo", dict(SUMO, round_time=30.0), dict(SUMO, round_time=0.2))
        self.assertTrue(any("reinício" in e for e in ev))

    def test_sem_evento_sem_mudanca(self):
        self.assertEqual(log_game.eventos("ctf", CTF, dict(CTF, round_time=4.0)), [])
        self.assertEqual(log_game.eventos("ctf", None, CTF), [])

    def test_gravador_escreve_estados_eventos_e_resumo(self):
        with tempfile.TemporaryDirectory() as d:
            caminho = os.path.join(d, "sub", "x.jsonl")  # a pasta intermediária é criada
            g = log_game.Gravador(caminho, "teste")
            g.on_state(SOCCER)
            g.on_state(dict(SOCCER, blue=1, time=11.0))
            g.fechar()
            with open(caminho, encoding="utf-8") as f:
                linhas = [json.loads(l) for l in f]
            self.assertTrue(any("state" in l for l in linhas))
            self.assertTrue(any("event" in l and "gol" in l["event"] for l in linhas))
            resumo = linhas[-1]["summary"]
            self.assertEqual(resumo["jogo"], "soccer")
            self.assertEqual(resumo["placar_final"], [1, 0])


def grava(pasta, nome, estado_final, eventos=1):
    p = Path(pasta)
    p.mkdir(parents=True, exist_ok=True)
    with open(p / nome, "w", encoding="utf-8") as f:
        f.write(json.dumps({"t": 1.0, "state": estado_final}) + "\n")
        for _ in range(eventos):
            f.write(json.dumps({"t": 2.0, "event": "x"}) + "\n")
        f.write(json.dumps({"summary": {"duracao_s": 300.0, "estado_final": estado_final}}) + "\n")


class AnalisadorTests(unittest.TestCase):
    def test_nome_fora_do_padrao_e_ignorado(self):
        with tempfile.TemporaryDirectory() as d:
            grava(Path(d) / "ctf", "qualquer.jsonl", dict(CTF, score=[1, 0]))
            self.assertIsNone(summarize.analisar(Path(d) / "ctf" / "qualquer.jsonl", "ctf"))

    def test_resultado_depende_do_lado(self):
        with tempfile.TemporaryDirectory() as d:
            azul = dict(CTF, score=[2, 1])
            grava(Path(d) / "ctf", "20261123-2000_v2_vs_tech_azul.jsonl", azul)
            grava(Path(d) / "ctf", "20261123-2010_v2_vs_tech_vermelho.jsonl", azul)
            a = summarize.analisar(Path(d) / "ctf" / "20261123-2000_v2_vs_tech_azul.jsonl", "ctf")
            v = summarize.analisar(Path(d) / "ctf" / "20261123-2010_v2_vs_tech_vermelho.jsonl", "ctf")
            self.assertEqual((a["feitos"], a["sofridos"], a["resultado"]), (2, 1, "V"))
            self.assertEqual((v["feitos"], v["sofridos"], v["resultado"]), (1, 2, "D"))

    def test_sumo_por_lado_e_empate(self):
        with tempfile.TemporaryDirectory() as d:
            grava(Path(d) / "sumo", "20261123-2000_v2_vs_v1_hero1.jsonl", dict(SUMO, wins=[1, 3], draws=1))
            r = summarize.analisar(Path(d) / "sumo" / "20261123-2000_v2_vs_v1_hero1.jsonl", "sumo")
            self.assertEqual((r["feitos"], r["sofridos"], r["resultado"], r["empates_de_rodada"]), (3, 1, "V", 1))
            grava(Path(d) / "soccer", "20261123-2000_v2_vs_nyx_azul.jsonl", dict(SOCCER))
            e = summarize.analisar(Path(d) / "soccer" / "20261123-2000_v2_vs_nyx_azul.jsonl", "soccer")
            self.assertEqual(e["resultado"], "E")

    def test_agregado(self):
        base = dict(jogo="ctf", nossa="v2", adv="tech")
        ps = [dict(base, feitos=2, sofridos=0, resultado="V"), dict(base, feitos=0, sofridos=1, resultado="D"),
              dict(base, feitos=1, sofridos=1, resultado="E")]
        ag = summarize.agregar(ps)
        self.assertEqual(len(ag), 1)
        self.assertEqual((ag[0]["partidas"], ag[0]["V"], ag[0]["E"], ag[0]["D"]), (3, 1, 1, 1))
        self.assertEqual((ag[0]["feitos"], ag[0]["sofridos"], ag[0]["saldo"]), (3, 2, 1))

    def test_arquivo_cortado_sem_resumo(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "ctf"
            p.mkdir()
            nome = p / "20261123-2000_v2_vs_v1_azul.jsonl"
            nome.write_text(json.dumps({"t": 1, "state": dict(CTF, score=[1, 0])}) + "\n" + '{"t": 2, "sta', encoding="utf-8")
            r = summarize.analisar(nome, "ctf")
            self.assertEqual(r["resultado"], "V")


if __name__ == "__main__":
    unittest.main(verbosity=2)
