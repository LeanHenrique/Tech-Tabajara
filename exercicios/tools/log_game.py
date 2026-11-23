#!/usr/bin/env python3
"""Logger de partidas: grava /game/state em JSONL e imprime um resumo ao sair.

FERRAMENTA LOCAL: nunca entra nos três arquivos de jogo (ela abre arquivos).

Uso (no container ROS, com um cenário rodando em outro terminal):

    ./run.sh shell
    python3 /ws/psr_exercicios/tools/log_game.py --out /ws/psr_exercicios/registros/sumo/20261123-2000_v2_vs_v1_hero0.jsonl --label "v2 x v1"

Só a pasta exercicios/ é montada no container, então a saída tem de ficar dentro de /ws/psr_exercicios.
Pare com Ctrl-C: o resumo é impresso e gravado na última linha do arquivo.

Formato do arquivo (uma linha JSON por registro):
    {"t": <segundos desde o início>, "wall": <epoch>, "state": {...}}      mudança de placar ou batimento de 1 s
    {"t": ..., "event": "texto"}                                           ponto, gol, rodada, reinício
    {"summary": {...}}                                                     última linha
"""
import argparse
import json
import os
import sys
import time

HEARTBEAT_S = 1.0  # grava um estado a cada 1 s mesmo sem mudança (para reconstruir a linha do tempo)
IGNORAR = ("round_time", "time", "field", "ring_radius", "base_x", "base_radius")


def detectar_jogo(state):
    """'sumo', 'ctf', 'soccer' ou None, pelas chaves do /game/state."""
    if "wins" in state:
        return "sumo"
    if "score" in state and "held" in state:
        return "ctf"
    if "blue" in state and "red" in state:
        return "soccer"
    return None


def placar(jogo, state):
    """Placar como tupla (lado 0, lado 1): sumô = vitórias de hero_0/hero_1; CTF e futebol = azul/vermelho."""
    if jogo == "sumo":
        return tuple(state.get("wins", (0, 0)))
    if jogo == "ctf":
        return tuple(state.get("score", (0, 0)))
    if jogo == "soccer":
        return (state.get("blue", 0), state.get("red", 0))
    return (0, 0)


def eventos(jogo, ant, atual):
    """Lista de textos dos eventos entre dois estados consecutivos."""
    out = []
    if ant is None or jogo is None:
        return out
    if jogo == "sumo":
        for i in (0, 1):
            if atual["wins"][i] > ant["wins"][i]:
                out.append("rodada: vitória do hero_%d (%d x %d)" % (i, atual["wins"][0], atual["wins"][1]))
        if atual.get("draws", 0) > ant.get("draws", 0):
            out.append("rodada: empate (%d)" % atual["draws"])
    elif jogo == "ctf":
        for i, nome in ((0, "azul"), (1, "vermelho")):
            if atual["score"][i] > ant["score"][i]:
                out.append("ponto do %s (%d x %d)" % (nome, atual["score"][0], atual["score"][1]))
        if list(atual.get("tags", [])) != list(ant.get("tags", [])):
            out.append("hold: tags %s" % (atual.get("tags"),))
    elif jogo == "soccer":
        for i, nome, k in ((0, "azul", "blue"), (1, "vermelho", "red")):
            if atual[k] > ant[k]:
                out.append("gol do %s (%d x %d)" % (nome, atual["blue"], atual["red"]))
    rt_a, rt_b = ant.get("round_time"), atual.get("round_time")
    if rt_a is not None and rt_b is not None and rt_b < rt_a - 0.5:
        out.append("reinício de rodada (round_time %.1f -> %.1f)" % (rt_a, rt_b))
    return out


def chave_de_mudanca(state):
    return json.dumps({k: v for k, v in state.items() if k not in IGNORAR}, sort_keys=True)


class Gravador:
    def __init__(self, caminho, rotulo):
        pasta = os.path.dirname(os.path.abspath(caminho))
        if not os.path.isdir(pasta):
            os.makedirs(pasta)
        self.f = open(caminho, "w")
        self.caminho, self.rotulo = caminho, rotulo
        self.t0 = time.time()
        self.jogo = None
        self.ant = None
        self.ultimo_batimento = -1e9
        self.ultima_chave = None
        self.eventos = []
        self.final = None

    def escreve(self, obj):
        self.f.write(json.dumps(obj) + "\n")
        self.f.flush()

    def on_state(self, state):
        agora = time.time()
        t = round(agora - self.t0, 2)
        if self.jogo is None:
            self.jogo = detectar_jogo(state)
            print("[log] jogo detectado: %s" % self.jogo)
        for texto in eventos(self.jogo, self.ant, state):
            self.eventos.append((t, texto))
            self.escreve({"t": t, "event": texto})
            print("[log] t=%6.1fs  %s" % (t, texto))
        chave = chave_de_mudanca(state)
        if chave != self.ultima_chave or agora - self.ultimo_batimento >= HEARTBEAT_S:
            self.escreve({"t": t, "wall": round(agora, 3), "state": state})
            self.ultima_chave, self.ultimo_batimento = chave, agora
        self.ant, self.final = state, state

    def fechar(self):
        dur = round(time.time() - self.t0, 1)
        resumo = {"label": self.rotulo, "jogo": self.jogo, "duracao_s": dur, "n_eventos": len(self.eventos),
                  "placar_final": list(placar(self.jogo, self.final)) if self.final else None,
                  "estado_final": self.final}
        self.escreve({"summary": resumo})
        self.f.close()
        print("\n[log] resumo: jogo=%s  duração=%.1fs  placar final=%s  eventos=%d\n[log] arquivo: %s"
              % (self.jogo, dur, resumo["placar_final"], len(self.eventos), self.caminho))


def main():
    import rospy
    from std_msgs.msg import String

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True, help="arquivo .jsonl de saída (dentro de /ws/psr_exercicios)")
    ap.add_argument("--label", default="", help="texto livre, vai para o resumo")
    args = ap.parse_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node("log_game", anonymous=True)
    g = Gravador(args.out, args.label)

    def cb(msg):
        try:
            g.on_state(json.loads(msg.data))
        except ValueError:
            pass

    rospy.Subscriber("/game/state", String, cb, queue_size=10)
    rospy.on_shutdown(g.fechar)
    print("[log] gravando /game/state em %s (Ctrl-C para parar)" % args.out)
    rospy.spin()


if __name__ == "__main__":
    main()
