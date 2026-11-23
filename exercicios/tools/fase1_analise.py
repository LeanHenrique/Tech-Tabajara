#!/usr/bin/env python3
"""Análise dos experimentos da Fase 1 (E1 a E3). FERRAMENTA LOCAL (fora do container, Python 3.8+).

    python3 exercicios/tools/fase1_analise.py brake [arquivo.csv]
    python3 exercicios/tools/fase1_analise.py push  [arquivo.csv ...]

Lê os CSVs de exercicios/registros/fase1/ (colunas: wall, rnd, rt, x0, y0, yaw0, x1, y1, yaw1, v, w, tag).
"""
import csv
import math
import sys
from pathlib import Path

PASTA = Path(__file__).resolve().parent.parent / "registros" / "fase1"


def ler(caminho):
    with open(caminho, encoding="utf-8") as f:
        linhas = list(csv.DictReader(f))
    for r in linhas:
        for k in ("wall", "x0", "y0", "yaw0", "x1", "y1", "yaw1", "v", "w"):
            r[k] = float(r[k])
        r["rnd"] = int(r["rnd"])
        r["rt"] = float(r["rt"]) if r["rt"] != "" else None
    return linhas


def trechos(linhas, prefixo):
    """{k: [linhas]} dos trechos cujo tag começa com prefixo_<k>_."""
    out = {}
    for r in linhas:
        if r["tag"].startswith(prefixo + "_"):
            k = int(r["tag"].split("_")[1])
            out.setdefault(k, []).append(r)
    return out


def freada(caminho):
    linhas = ler(caminho)
    acc, brk = trechos(linhas, "acc"), trechos(linhas, "brk")
    res = []
    for k in sorted(brk):
        a, b = acc.get(k, []), brk[k]
        v_cmd = float(b[0]["tag"].split("_v")[1])
        # velocidade no fim da aceleração: deslocamento nos últimos ~0,4 s antes de frear
        fim = [r for r in a if r["wall"] >= a[-1]["wall"] - 0.4]
        dt = fim[-1]["wall"] - fim[0]["wall"]
        d_acc = math.hypot(fim[-1]["x0"] - fim[0]["x0"], fim[-1]["y0"] - fim[0]["y0"])
        v_real = d_acc / dt if dt > 0 else float("nan")
        x0, y0 = b[0]["x0"], b[0]["y0"]
        # a odometria é publicada a 25 Hz: a posição "parada" é a do fim do trecho
        d = math.hypot(b[-1]["x0"] - x0, b[-1]["y0"] - y0)
        t = b[-1]["wall"] - b[0]["wall"]
        # tempo até a velocidade cair abaixo de 5% da inicial (por diferenças de posição em janelas de 0,12 s)
        t95 = float("nan")
        for i in range(len(b) - 3):
            seg = b[i + 3]["wall"] - b[i]["wall"]
            if seg > 0:
                v = math.hypot(b[i + 3]["x0"] - b[i]["x0"], b[i + 3]["y0"] - b[i]["y0"]) / seg
                if v < 0.05 * abs(v_cmd):
                    t95 = b[i]["wall"] - b[0]["wall"]
                    break
        res.append(dict(k=k, v_cmd=v_cmd, v_real=v_real, dist_m=d, t_total_s=t, t_ate_5pct_s=t95))
    return res


CONTATO = 0.085  # m: distância entre centros abaixo da qual há contato (2 raios = 0,074)
RAIO = 0.40


def segmentos(linhas):
    """Divide em rodadas: nova rodada quando o round_time cai (teleporte para o nascimento)."""
    segs, atual, rt_ant = [], [], None
    for r in linhas:
        if r["rt"] is None:
            continue
        if rt_ant is not None and r["rt"] < rt_ant - 1.0 and atual:
            segs.append(atual)
            atual = []
        atual.append(r)
        rt_ant = r["rt"]
    if atual:
        segs.append(atual)
    return segs


def empurrao(caminho):
    """Por rodada com contato: velocidade do par, deslocamento do adversário, quem caiu e em quanto tempo."""
    res = []
    for n, seg in enumerate(segmentos(ler(caminho))):
        cont = [r for r in seg if math.hypot(r["x0"] - r["x1"], r["y0"] - r["y1"]) <= CONTATO]
        if not cont:
            continue
        t0 = cont[0]["wall"]
        r0max = max(math.hypot(r["x0"], r["y0"]) for r in seg)
        r1max = max(math.hypot(r["x1"], r["y1"]) for r in seg)
        queda = next((r for r in seg if r["wall"] >= t0 and math.hypot(r["x1"], r["y1"]) > RAIO), None)
        fim = queda or seg[-1]
        # velocidade do par: do adversário entre 0,3 s após o contato e o fim (ou a queda)
        jan = [r for r in seg if t0 + 0.3 <= r["wall"] <= fim["wall"]]
        v_par = float("nan")
        if len(jan) >= 2 and jan[-1]["wall"] > jan[0]["wall"]:
            v_par = math.hypot(jan[-1]["x1"] - jan[0]["x1"], jan[-1]["y1"] - jan[0]["y1"]) / (jan[-1]["wall"] - jan[0]["wall"])
        c = cont[0]
        res.append(dict(rodada=n, dur_contato_s=fim["wall"] - t0, v_par=v_par,
                        dx_adv=fim["x1"] - c["x1"], dy_adv=fim["y1"] - c["y1"],
                        adv_caiu=r1max > RAIO, eu_caí=r0max > RAIO,
                        tempo_ate_queda_s=(queda["wall"] - t0) if queda else float("nan")))
    return res


def main():
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    modo = sys.argv[1]
    if modo == "push":
        arquivos = sys.argv[2:] or [str(p) for p in sorted(PASTA.glob("exp_push_*.csv"))]
        for a in arquivos:
            res = empurrao(a)
            print("\n== %s (%d rodadas com contato)" % (Path(a).name, len(res)))
            print("%-4s %-10s %-9s %-9s %-9s %-9s %-6s %s" % ("rod", "contato(s)", "v_par", "dx(cm)", "dy(cm)", "queda(s)", "adv", "eu"))
            for r in res:
                print("%-4d %-10.2f %-9.3f %-9.1f %-9.1f %-9.2f %-6s %s" % (
                    r["rodada"], r["dur_contato_s"], r["v_par"], 100 * r["dx_adv"], 100 * r["dy_adv"],
                    r["tempo_ate_queda_s"], "caiu" if r["adv_caiu"] else "-", "caí" if r["eu_caí"] else "-"))
            vs = [r["v_par"] for r in res if r["v_par"] == r["v_par"]]
            if vs:
                print("velocidade do par em contato: média %.3f m/s (min %.3f, max %.3f); adversário caiu em %d/%d rodadas" % (
                    sum(vs) / len(vs), min(vs), max(vs), sum(r["adv_caiu"] for r in res), len(res)))
        return
    if modo == "brake":
        caminho = sys.argv[2] if len(sys.argv) > 2 else PASTA / "exp_brake.csv"
        res = freada(caminho)
        print("%-3s %-8s %-10s %-11s %-12s %s" % ("k", "v_cmd", "v_real", "dist (cm)", "t_total (s)", "t até 5% (s)"))
        for r in res:
            print("%-3d %+.2f    %.3f      %6.2f      %6.2f       %6.2f" % (
                r["k"], r["v_cmd"], r["v_real"], 100 * r["dist_m"], r["t_total_s"], r["t_ate_5pct_s"]))
        for v in sorted({abs(r["v_cmd"]) for r in res}):
            ds = [r["dist_m"] for r in res if abs(r["v_cmd"]) == v]
            print("v = %.2f m/s: distância de frenagem média %.2f cm (min %.2f, max %.2f, n = %d)" % (
                v, 100 * sum(ds) / len(ds), 100 * min(ds), 100 * max(ds), len(ds)))
    else:
        raise SystemExit("modo desconhecido: %s" % modo)


if __name__ == "__main__":
    main()
