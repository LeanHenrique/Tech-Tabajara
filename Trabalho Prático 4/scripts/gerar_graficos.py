#!/usr/bin/env python3
"""Refaz os gráficos e as tabelas do relatório a partir dos CSVs gravados.

Uso: python scripts/gerar_graficos.py registros/escada_*.csv registros/inversao_*.csv ...
     (aceita também neutro_*, aberta_*, degrau_*, relatorio_*)

Saída em figuras/:
  <arquivo>.png           gráfico do ensaio
  <arquivo>_tabela.csv    números usados no texto (patamares, filtros, curva)
O tipo do ensaio vem da coluna "ensaio" (escrita pelo firmware).
"""

import glob
import re
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

PASTA_SAIDA = Path("figuras")
LIMIAR_PARADO = 2.0  # mm/s: abaixo disso a roda é considerada parada (banda morta)


# ------------------------------------------------------------------ leitura
def ler_registro(caminho):
    cond = {}
    with open(caminho, encoding="utf-8") as f:
        for linha in f:
            if linha.startswith("# cond"):
                for chave, valor in re.findall(r"(\w+)=(\S+)", linha):
                    cond[chave] = valor
    df = pd.read_csv(caminho, comment="#")
    df = df.dropna(subset=["t_s"]).reset_index(drop=True)
    return df, cond


def texto_condicao(cond):
    filtro = cond.get("filtro", "?")
    par = f"α={cond.get('alfa')}" if filtro == "EMA" else f"janela={cond.get('janela')}"
    return (
        f"neutro={cond.get('neutro')} µs · k={cond.get('k')} mm/tick · Δt={cond.get('dt_ms')} ms · "
        f"kp={cond.get('kp')} ki={cond.get('ki')} kd={cond.get('kd')} · filtro {filtro} ({par})"
    )


def segmentos(serie):
    """Índices [ini, fim) de trechos contíguos com o mesmo valor."""
    muda = np.flatnonzero(serie.values[1:] != serie.values[:-1]) + 1
    bordas = np.concatenate(([0], muda, [len(serie)]))
    return list(zip(bordas[:-1], bordas[1:]))


def salvar(fig, df_tabela, nome):
    PASTA_SAIDA.mkdir(exist_ok=True)
    fig.savefig(PASTA_SAIDA / f"{nome}.png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    if df_tabela is not None:
        df_tabela.to_csv(PASTA_SAIDA / f"{nome}_tabela.csv", index=False)
        print(df_tabela.to_string(index=False))
    print(f"-> {PASTA_SAIDA / nome}.png\n")


# ------------------------------------------------------------- malha fechada
def analisar_patamares(df, cond):
    neutro = float(cond["neutro"])
    u_max = float(cond.get("U_MAX", 250))
    linhas = []
    for ini, fim in segmentos(df["setpoint"]):
        sp = df["setpoint"].iloc[ini]
        if sp == 0:
            continue
        trecho = df.iloc[ini:fim]
        t0 = trecho["t_s"].iloc[0]
        dur = trecho["t_s"].iloc[-1] - t0
        regime = trecho[trecho["t_s"] >= t0 + dur / 2]  # segunda metade do patamar
        v0 = df["v_pid"].iloc[max(ini - 1, 0)]
        amp = sp - v0
        rel = (trecho["v_pid"] - v0) / amp  # 0 no início, 1 no setpoint

        def tempo_ate(frac):
            ok = np.flatnonzero(rel.values >= frac)
            return trecho["t_s"].iloc[ok[0]] - t0 if len(ok) else np.nan

        t10, t90 = tempo_ate(0.1), tempo_ate(0.9)
        sobressinal = max(0.0, (rel.max() - 1) * 100)
        sat = (np.abs(trecho["pulso"] - neutro) >= u_max - 1).mean() * 100

        # inversão: tempo em que a roda ficou parada ao passar pelo zero
        parado = np.nan
        if v0 * sp < 0 and abs(v0) > LIMIAR_PARADO:
            sinal_novo = np.flatnonzero(np.sign(trecho["v_bruta"].values) == np.sign(sp))
            cruzou = np.flatnonzero(np.sign(trecho["v_bruta"].values) != np.sign(v0))
            if len(sinal_novo) and len(cruzou):
                parado = trecho["t_s"].iloc[sinal_novo[0]] - trecho["t_s"].iloc[cruzou[0]]

        linhas.append({
            "setpoint_mm_s": sp,
            "v_media_mm_s": round(regime["v_pid"].mean(), 2),
            "erro_regime_mm_s": round(sp - regime["v_pid"].mean(), 2),
            "dp_bruta_mm_s": round(regime["v_bruta"].std(), 2),
            "dp_filtrada_mm_s": round(regime["v_pid"].std(), 2),
            "t_subida_10_90_s": round(t90 - t10, 2),
            "t_ate_90_s": round(t90, 2),
            "sobressinal_pct": round(sobressinal, 1),
            "pulso_medio_us": round(regime["pulso"].mean(), 0),
            "pulso_menos_neutro_us": round(regime["pulso"].mean() - neutro, 0),
            "saturado_pct": round(sat, 1),
            "tempo_parado_inversao_s": round(parado, 2) if not np.isnan(parado) else "",
        })
    return pd.DataFrame(linhas)


def grafico_malha_fechada(df, cond, nome):
    neutro = float(cond["neutro"])
    u_max = float(cond.get("U_MAX", 250))
    fig, (ax_v, ax_p, ax_d) = plt.subplots(
        3, 1, figsize=(10, 9), sharex=True, gridspec_kw={"height_ratios": [2.2, 1, 1]}
    )
    t = df["t_s"]
    ax_v.plot(t, df["v_bruta"], color="#BBBBBB", lw=0.8, label="Velocidade bruta")
    ax_v.plot(t, df["v_pid"], color="#2980B9", lw=1.6, label="Velocidade filtrada (PID)")
    ax_v.step(t, df["setpoint"], where="post", color="#C0392B", lw=1.4, ls="--", label="Setpoint")
    ax_v.axhline(0, color="black", lw=0.6)
    ax_v.set_ylabel("Velocidade (mm/s)")
    ax_v.legend(loc="best", fontsize=8)
    ax_v.set_title(f"{nome}\n{texto_condicao(cond)}", fontsize=9)

    ax_p.plot(t, df["pulso"], color="#8E44AD", lw=1.2, label="Pulso comandado")
    ax_p.axhline(neutro, color="gray", lw=0.8, ls=":", label=f"Neutro ({neutro:.0f} µs)")
    for j, lim in enumerate((neutro - u_max, neutro + u_max)):
        ax_p.axhline(lim, color="#C0392B", lw=0.8, ls="--",
                     label=f"Neutro ± {u_max:.0f} µs" if j == 0 else None)
    ax_p.set_ylabel("Pulso (µs)")
    ax_p.legend(loc="best", fontsize=8)

    ax_d.plot(t, df["dist_mm"], color="#27AE60", lw=1.6)
    ax_d.set_ylabel("Distância (mm)")
    ax_d.set_xlabel("Tempo (s)")
    for eixo in (ax_v, ax_p, ax_d):
        eixo.grid(True, ls="--", alpha=0.4)

    tabela = analisar_patamares(df, cond)
    dist_final = df["dist_mm"].iloc[-1]
    voltas = df["ticks"].iloc[-1] / float(cond["N"])
    print(f"[{nome}] distância final = {dist_final:.1f} mm "
          f"({voltas:.2f} voltas da roda; máx. {df['dist_mm'].max():.1f} mm)")
    salvar(fig, tabela, nome)


# -------------------------------------------------------- filtros (degrau)
def grafico_degrau(df, cond, nome):
    fig, (ax_v, ax_p) = plt.subplots(
        2, 1, figsize=(10, 6.5), sharex=True, gridspec_kw={"height_ratios": [2.2, 1]}
    )
    t = df["t_s"]
    ax_v.plot(t, df["v_bruta"], color="#BBBBBB", lw=0.9, label="Bruta")
    ax_v.plot(t, df["v_ema"], color="#2980B9", lw=1.6, label=f"Passa-baixa 1ª ordem (α={cond.get('alfa')})")
    ax_v.plot(t, df["v_mm"], color="#E67E22", lw=1.6, label=f"Média móvel (janela={cond.get('janela')})")
    ax_v.set_ylabel("Velocidade (mm/s)")
    ax_v.legend(loc="best", fontsize=8)
    ax_v.set_title(f"Comparação de filtros no mesmo degrau\n{texto_condicao(cond)}", fontsize=9)
    ax_p.plot(t, df["pulso"], color="#8E44AD", lw=1.2)
    ax_p.set_ylabel("Pulso (µs)")
    ax_p.set_xlabel("Tempo (s)")
    for eixo in (ax_v, ax_p):
        eixo.grid(True, ls="--", alpha=0.4)

    # trecho do degrau
    degrau = df[df["fase"] == "degrau"]
    t0, t1 = degrau["t_s"].iloc[0], degrau["t_s"].iloc[-1]
    regime = degrau[degrau["t_s"] >= t1 - 3.0]
    v_final = regime["v_bruta"].mean()
    linhas = []
    for col, nome_f in (("v_bruta", "bruta"), ("v_ema", "passa-baixa"), ("v_mm", "media movel")):
        sub = degrau[col].values
        ok = np.flatnonzero(sub >= 0.9 * v_final) if v_final > 0 else np.flatnonzero(sub <= 0.9 * v_final)
        t90 = degrau["t_s"].iloc[ok[0]] - t0 if len(ok) else np.nan
        # descida depois do degrau: tempo até ficar abaixo do limiar
        depois = df[df["t_s"] > t1]
        abaixo = np.flatnonzero(np.abs(depois[col].values) < LIMIAR_PARADO)
        t_desc = depois["t_s"].iloc[abaixo[0]] - t1 if len(abaixo) else np.nan
        linhas.append({
            "sinal": nome_f,
            "dp_regime_mm_s": round(regime[col].std(), 2),
            "media_regime_mm_s": round(regime[col].mean(), 2),
            "t_ate_90_s": round(t90, 2),
            f"t_ate_<{LIMIAR_PARADO:g}mm_s_apos_degrau_s": round(t_desc, 2),
        })
    tabela = pd.DataFrame(linhas)
    tabela["atraso_vs_bruta_s"] = (tabela["t_ate_90_s"] - tabela["t_ate_90_s"].iloc[0]).round(2)
    salvar(fig, tabela, nome)


# --------------------------------------------- malha aberta / busca do neutro
def curva_varredura(df, cond):
    descarte = float(cond.get("descarte_ms", 1000)) / 1000.0
    linhas = []
    for ini, fim in segmentos(df["pulso"]):
        trecho = df.iloc[ini:fim]
        t0 = trecho["t_s"].iloc[0]
        med = trecho[trecho["t_s"] >= t0 + descarte]
        if len(med) < 3:
            continue
        dt = med["t_s"].iloc[-1] - med["t_s"].iloc[0]
        v = (med["dist_mm"].iloc[-1] - med["dist_mm"].iloc[0]) / dt
        linhas.append({"pulso_us": int(trecho["pulso"].iloc[0]), "v_mm_s": round(v, 2),
                       "dp_bruta_mm_s": round(med["v_bruta"].std(), 2)})
    return pd.DataFrame(linhas).groupby("pulso_us", as_index=False).mean().round(2)


def grafico_varredura(df, cond, nome, busca_neutro):
    neutro = float(cond["neutro"])
    curva = curva_varredura(df, cond)
    parado = curva[np.abs(curva["v_mm_s"]) < LIMIAR_PARADO]

    fig, ax = plt.subplots(figsize=(9, 5.5))
    ax.plot(curva["pulso_us"], curva["v_mm_s"], "o-", color="#2980B9", ms=4)
    ax.axvline(neutro, color="#C0392B", ls="--", lw=1.2, label=f"Neutro usado ({neutro:.0f} µs)")
    if len(parado):
        ax.axvspan(parado["pulso_us"].min(), parado["pulso_us"].max(), color="#F1C40F", alpha=0.3,
                   label=f"Banda morta (|v|<{LIMIAR_PARADO:g} mm/s): "
                         f"{parado['pulso_us'].min()}–{parado['pulso_us'].max()} µs")
    ax.axhline(0, color="black", lw=0.6)
    ax.set_xlabel("Pulso (µs)")
    ax.set_ylabel("Velocidade em regime (mm/s)")
    titulo = "Busca do neutro" if busca_neutro else "Curva pulso × velocidade em malha aberta"
    ax.set_title(f"{titulo}\nneutro={cond.get('neutro')} µs · k={cond.get('k')} mm/tick · "
                 f"descarte={cond.get('descarte_ms')} ms por pulso", fontsize=9)
    ax.legend(fontsize=8)
    ax.grid(True, ls="--", alpha=0.4)

    if len(parado):
        centro = (parado["pulso_us"].min() + parado["pulso_us"].max()) / 2
        print(f"[{nome}] banda morta: {parado['pulso_us'].min()}–{parado['pulso_us'].max()} µs "
              f"-> neutro sugerido (centro) = {centro:.0f} µs")
    if not busca_neutro:
        print(f"[{nome}] v(1000 µs) = {curva['v_mm_s'].iloc[0]:.1f} mm/s, "
              f"v(2000 µs) = {curva['v_mm_s'].iloc[-1]:.1f} mm/s")
        # assimetria: mesma distância ao neutro nos dois sentidos
        assim = []
        for delta in (50, 100, 150, 200, 250, 300, 400, 500):
            mais = curva[curva["pulso_us"] == min(curva["pulso_us"], key=lambda p: abs(p - (neutro + delta)))]
            menos = curva[curva["pulso_us"] == min(curva["pulso_us"], key=lambda p: abs(p - (neutro - delta)))]
            assim.append({"delta_us": delta,
                          "v_pos_mm_s": float(mais["v_mm_s"].iloc[0]),
                          "v_neg_mm_s": float(menos["v_mm_s"].iloc[0])})
        tab_a = pd.DataFrame(assim)
        tab_a["razao_pos_neg"] = (tab_a["v_pos_mm_s"] / -tab_a["v_neg_mm_s"]).round(2)
        print("Assimetria entre sentidos:")
        print(tab_a.to_string(index=False))
        PASTA_SAIDA.mkdir(exist_ok=True)
        tab_a.to_csv(PASTA_SAIDA / f"{nome}_assimetria.csv", index=False)
    salvar(fig, curva, nome)


# ------------------------------------------------------------------- main
def main(arquivos):
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except AttributeError:
        pass
    if not arquivos:
        print(__doc__)
        sys.exit(1)
    # o PowerShell/cmd não expande "registros/*.csv"; expandimos aqui
    expandidos = []
    for a in arquivos:
        expandidos += sorted(glob.glob(a)) if any(c in a for c in "*?[") else [a]
    for caminho in expandidos:
        df, cond = ler_registro(caminho)
        if df.empty or "neutro" not in cond:
            print(f"{caminho}: vazio ou sem linha '# cond', ignorado")
            continue
        nome = Path(caminho).stem
        tipo = str(df["ensaio"].iloc[0])
        print(f"== {caminho} ({tipo})")
        if tipo == "manual":
            print("registro manual, ignorado\n")
        elif tipo in ("escada", "inversao", "relatorio"):
            grafico_malha_fechada(df, cond, nome)
        elif tipo == "degrau":
            grafico_degrau(df, cond, nome)
        elif tipo in ("neutro", "aberta"):
            grafico_varredura(df, cond, nome, busca_neutro=(tipo == "neutro"))
        else:
            print("tipo desconhecido, ignorado")


if __name__ == "__main__":
    main(sys.argv[1:])
