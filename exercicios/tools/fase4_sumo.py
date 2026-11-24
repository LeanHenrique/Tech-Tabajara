#!/usr/bin/env python3
"""Análise de um registro do watch_sumo.py (exercicios/registros/fase4/*.txt). FERRAMENTA LOCAL.

    python3 exercicios/tools/fase4_sumo.py <arquivo.txt> [-v]

Divide o registro em rodadas (nova rodada quando o round_time cai), diz quem caiu e quando, e com -v mostra a
trajetória dos dois robôs nos instantes antes da queda. O robô 0 é o nosso nos testes desta fase.
"""
import math
import re
import sys

LINHA = re.compile(
    r"rt=\s*(?P<rt>[\d.]+) wins=\[(?P<w0>\d+), (?P<w1>\d+)\] \| "
    r"h0\((?P<x0>[+-][\d.]+),(?P<y0>[+-][\d.]+),\s*(?P<a0>[+-]?\d+) r=(?P<r0>[\d.]+)\) \| "
    r"h1\((?P<x1>[+-][\d.]+),(?P<y1>[+-][\d.]+),\s*(?P<a1>[+-]?\d+) r=(?P<r1>[\d.]+)\) \| dist=(?P<d>[\d.]+)")
RAIO = 0.40


def ler(caminho):
    pts = []
    with open(caminho, encoding="utf-8", errors="replace") as f:
        for linha in f:
            m = LINHA.search(linha)
            if m:
                d = {k: float(v) for k, v in m.groupdict().items()}
                d["w0"], d["w1"] = int(d["w0"]), int(d["w1"])
                pts.append(d)
    return pts


def rodadas(pts):
    segs, atual, rt_ant = [], [], None
    for p in pts:
        if rt_ant is not None and p["rt"] < rt_ant - 1.0 and atual:
            segs.append(atual)
            atual = []
        atual.append(p)
        rt_ant = p["rt"]
    if atual:
        segs.append(atual)
    return segs


def resultado(seg, seg_seguinte):
    """Quem caiu na rodada: usa o placar do início da seguinte (ou o fim desta) e as posições."""
    r0 = max(p["r0"] for p in seg)
    r1 = max(p["r1"] for p in seg)
    if r0 > RAIO and r1 > RAIO:
        return "empate (os dois caíram)"
    if r0 > RAIO:
        return "NOSSO robô caiu"
    if r1 > RAIO:
        return "adversário caiu"
    return "empate por tempo"


def primeira_saida(seg, chave):
    for i, p in enumerate(seg):
        if p[chave] > RAIO:
            return i
    return None


def main():
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    pts = ler(sys.argv[1])
    segs = rodadas(pts)
    verbose = "-v" in sys.argv
    print("%d amostras, %d rodadas (a primeira e a última podem estar cortadas)" % (len(pts), len(segs)))
    cont = {}
    for n, seg in enumerate(segs):
        res = resultado(seg, segs[n + 1] if n + 1 < len(segs) else None)
        cont[res] = cont.get(res, 0) + 1
        dur = seg[-1]["rt"] - seg[0]["rt"]
        i0, i1 = primeira_saida(seg, "r0"), primeira_saida(seg, "r1")
        quando = ""
        if i0 is not None or i1 is not None:
            i = min(x for x in (i0, i1) if x is not None)
            quando = " | queda em rt=%.1f s" % seg[i]["rt"]
        print("rodada %2d: %-26s duração observada %5.1f s%s" % (n, res, dur, quando))
        if verbose and res == "NOSSO robô caiu":
            i = i0
            print("    trajetória até a queda (rt, nosso x,y,yaw r | adversário x,y,yaw r | dist):")
            for p in seg[max(0, i - 25): i + 2]:
                print("    rt=%5.1f  h0(%+.2f,%+.2f,%+4.0f r=%.2f)  h1(%+.2f,%+.2f,%+4.0f r=%.2f)  dist=%.2f" % (
                    p["rt"], p["x0"], p["y0"], p["a0"], p["r0"], p["x1"], p["y1"], p["a1"], p["r1"], p["d"]))
    print("\nresumo:", cont)


if __name__ == "__main__":
    main()
