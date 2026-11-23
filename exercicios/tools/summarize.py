#!/usr/bin/env python3
"""Analisador dos registros do log_game.py: tabelas V/E/D e pontos feitos/sofridos, em Markdown e CSV.

FERRAMENTA LOCAL (roda fora do container, com Python 3.8+; só biblioteca padrão).

    python3 exercicios/tools/summarize.py [--dir exercicios/registros] [--out exercicios/registros/tabelas]

Nome esperado dos arquivos:   <AAAAMMDD-HHMM>_<nossa>_vs_<adversária>_<lado>.jsonl
  lado: sumô = hero0 | hero1;  CTF e futebol = azul | vermelho.
  Ex.: 20261123-2000_v2_vs_v1_hero0.jsonl, em registros/sumo/.
A pasta de cada jogo (sumo, ctf, soccer) diz de que jogo é o arquivo.

Resultado de uma partida:
  sumô: quem tem mais rodadas vencidas (empates de rodada à parte);
  CTF e futebol: quem tem mais pontos/gols no placar final.
"""
import argparse
import csv
import json
import re
import sys
from collections import OrderedDict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import log_game  # noqa: E402  (só usa a parte pura do logger; rospy é importado apenas no main)

NOME = re.compile(r"^(?P<data>\d{8}-\d{4})_(?P<nossa>.+?)_vs_(?P<adv>.+)_(?P<lado>hero0|hero1|azul|vermelho)$")
LADO_IDX = {"hero0": 0, "azul": 0, "hero1": 1, "vermelho": 1}


def ler(caminho):
    """(resumo, ultimo_estado, n_eventos) de um .jsonl; tolera arquivo cortado (sem resumo)."""
    resumo, estado, eventos = None, None, 0
    with open(caminho, encoding="utf-8") as f:
        for linha in f:
            linha = linha.strip()
            if not linha:
                continue
            try:
                obj = json.loads(linha)
            except ValueError:
                continue  # última linha cortada
            if "summary" in obj:
                resumo = obj["summary"]
            elif "state" in obj:
                estado = obj["state"]
            elif "event" in obj:
                eventos += 1
    return resumo, estado, eventos


def analisar(caminho, jogo_da_pasta):
    m = NOME.match(Path(caminho).stem)
    if not m:
        return None
    resumo, estado, eventos = ler(caminho)
    estado = (resumo or {}).get("estado_final") or estado
    if not estado:
        return None
    jogo = log_game.detectar_jogo(estado) or jogo_da_pasta
    a, b = log_game.placar(jogo, estado)
    nosso = LADO_IDX[m["lado"]]
    feitos, sofridos = (a, b) if nosso == 0 else (b, a)
    res = "V" if feitos > sofridos else "D" if feitos < sofridos else "E"
    return OrderedDict(jogo=jogo, arquivo=Path(caminho).name, data=m["data"], nossa=m["nossa"], adv=m["adv"],
                       lado=m["lado"], feitos=feitos, sofridos=sofridos, resultado=res,
                       empates_de_rodada=estado.get("draws", 0) if jogo == "sumo" else "",
                       duracao_s=(resumo or {}).get("duracao_s", ""), eventos=eventos)


def agregar(partidas):
    """Por (jogo, nossa, adv): n, V/E/D, feitos, sofridos, saldo."""
    ag = OrderedDict()
    for p in partidas:
        k = (p["jogo"], p["nossa"], p["adv"])
        a = ag.setdefault(k, OrderedDict(jogo=k[0], nossa=k[1], adv=k[2], partidas=0, V=0, E=0, D=0,
                                         feitos=0, sofridos=0))
        a["partidas"] += 1
        a[p["resultado"]] += 1
        a["feitos"] += p["feitos"]
        a["sofridos"] += p["sofridos"]
    for a in ag.values():
        a["saldo"] = a["feitos"] - a["sofridos"]
    return list(ag.values())


def md(linhas, colunas):
    out = ["| " + " | ".join(colunas) + " |", "|" + "|".join(["---"] * len(colunas)) + "|"]
    out += ["| " + " | ".join(str(l[c]) for c in colunas) + " |" for l in linhas]
    return "\n".join(out)


def main():
    raiz = Path(__file__).resolve().parent.parent
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir", default=str(raiz / "registros"))
    ap.add_argument("--out", default=str(raiz / "registros" / "tabelas"))
    args = ap.parse_args()

    partidas = []
    for jogo in ("sumo", "ctf", "soccer"):
        for p in sorted((Path(args.dir) / jogo).glob("*.jsonl")):
            r = analisar(p, jogo)
            if r is None:
                print("aviso: ignorado %s (nome fora do padrão ou sem estado)" % p.name)
            else:
                partidas.append(r)
    if not partidas:
        raise SystemExit("nenhum registro .jsonl encontrado em %s/<jogo>/" % args.dir)

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    cols_p = list(partidas[0].keys())
    agregado = agregar(partidas)
    cols_a = list(agregado[0].keys())
    for nome, linhas, cols in (("partidas", partidas, cols_p), ("resumo", agregado, cols_a)):
        with open(out / ("%s.csv" % nome), "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=cols)
            w.writeheader()
            w.writerows(linhas)
    texto = "## Resumo por confronto\n\n%s\n\n## Partidas\n\n%s\n" % (md(agregado, cols_a), md(partidas, cols_p))
    (out / "tabelas.md").write_text(texto, encoding="utf-8")
    print(texto)
    print("gravado em %s" % out)


if __name__ == "__main__":
    main()
