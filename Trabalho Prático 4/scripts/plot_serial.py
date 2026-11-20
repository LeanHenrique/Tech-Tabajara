#!/usr/bin/env python3
"""Lê a serial da Mega, plota ao vivo e grava um CSV por ensaio.

Comandos digitados no terminal vão para a placa (ex.: "b", "a", "g 150",
"e", "i", "r", "x", "v 40", "kp 0.3", "f 1").

Arquivos gerados em PASTA (padrão: registros/):
  <ensaio>_AAAAMMDD_HHMMSS.csv  -> uma linha por amostra; as linhas "# cond"
                                   (neutro, k, Δt, ganhos, filtro) e "# MEDIDA"
                                   ficam no início/meio do arquivo como comentário
  eventos_AAAAMMDD_HHMMSS.txt   -> todas as mensagens "#" da placa

Uso: python scripts/plot_serial.py [PORTA] [PASTA]
Depois: python scripts/gerar_graficos.py registros/*.csv
"""

import re
import sys
import threading
import time
from pathlib import Path

import matplotlib.pyplot as plt
import serial

porta = sys.argv[1] if len(sys.argv) > 1 else "COM7"
pasta = Path(sys.argv[2]) if len(sys.argv) > 2 else Path("registros")
pasta.mkdir(parents=True, exist_ok=True)

ser = serial.Serial(porta, 115200, timeout=0.05)
time.sleep(2)  # a Mega reinicia ao abrir a porta

carimbo = time.strftime("%Y%m%d_%H%M%S")
arquivo_eventos = open(pasta / f"eventos_{carimbo}.txt", "w", encoding="utf-8")

cabecalho = None       # colunas enviadas pela placa
arq_atual = None       # CSV do ensaio em andamento
cabecalho_escrito = False

# ---------------------------------------------------------------- gráfico
plt.ion()
fig, (ax_v, ax_p, ax_d) = plt.subplots(
    3, 1, figsize=(9, 8), sharex=True, gridspec_kw={"height_ratios": [2, 1, 1]}
)
(l_bruta,) = ax_v.plot([], [], color="#BBBBBB", lw=1.0, label="Bruta")
(l_ema,) = ax_v.plot([], [], color="#2980B9", lw=1.6, label="Passa-baixa (EMA)")
(l_mm,) = ax_v.plot([], [], color="#E67E22", lw=1.2, label="Média móvel")
(l_sp,) = ax_v.plot([], [], color="#C0392B", lw=1.5, ls="--", label="Setpoint")
(l_pulso,) = ax_p.plot([], [], color="#8E44AD", lw=1.4)
(l_dist,) = ax_d.plot([], [], color="#27AE60", lw=1.6)

ax_v.set_ylabel("Velocidade (mm/s)")
ax_v.legend(loc="upper right", fontsize=8)
ax_p.set_ylabel("Pulso (µs)")
ax_p.set_ylim(950, 2050)
ax_d.set_ylabel("Distância (mm)")
ax_d.set_xlabel("Tempo (s)")
for eixo in (ax_v, ax_p, ax_d):
    eixo.grid(True, ls="--", alpha=0.4)
fig.tight_layout()

dados = {c: [] for c in ("t", "bruta", "ema", "mm", "sp", "pulso", "dist")}


def limpar_grafico(titulo):
    for lista in dados.values():
        lista.clear()
    ax_v.set_title(titulo)


def redesenhar():
    if not dados["t"]:
        return
    t = dados["t"]
    l_bruta.set_data(t, dados["bruta"])
    l_ema.set_data(t, dados["ema"])
    l_mm.set_data(t, dados["mm"])
    l_sp.set_data(t, dados["sp"])
    l_pulso.set_data(t, dados["pulso"])
    l_dist.set_data(t, dados["dist"])
    ax_v.set_xlim(t[0], max(t[-1], t[0] + 1))
    for eixo in (ax_v, ax_d):
        eixo.relim()
        eixo.autoscale_view(scalex=False)
    fig.canvas.draw_idle()
    fig.canvas.flush_events()


# ---------------------------------------------------------------- arquivos
def fechar_registro():
    global arq_atual, cabecalho_escrito
    if arq_atual is not None:
        arq_atual.close()
        print(f">> registro fechado: {arq_atual.name}")
    arq_atual = None
    cabecalho_escrito = False


def abrir_registro(nome):
    global arq_atual, cabecalho_escrito
    fechar_registro()
    caminho = pasta / f"{nome}_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    arq_atual = open(caminho, "w", newline="", encoding="utf-8")
    cabecalho_escrito = False
    print(f">> gravando {caminho}")
    limpar_grafico(nome)


def ler_teclado():
    """Roda em paralelo: cada linha digitada no terminal vai para a placa."""
    while True:
        try:
            comando = input()
        except EOFError:
            return
        if comando.strip():
            ser.write((comando.strip() + "\n").encode())


threading.Thread(target=ler_teclado, daemon=True).start()
print("Comandos: b (neutro), a (malha aberta), g 150 (degrau p/ filtros),")
print("          e (escada), i (inversão), r (e+i), x (parar), v 40, kp 0.3, f 1 ...")
print("Ctrl+C sai.")

n = 0
try:
    while True:
        bruto = ser.readline().decode(errors="ignore").strip()
        if not bruto:
            fig.canvas.flush_events()
            continue

        # ---- eventos
        if bruto.startswith("#"):
            print(bruto)
            arquivo_eventos.write(bruto + "\n")
            arquivo_eventos.flush()
            m = re.match(r"#\s*INICIO ensaio=(\w+)", bruto)
            if m:
                abrir_registro(m.group(1))
                continue
            if arq_atual is not None:
                arq_atual.write(bruto + "\n")
            if re.match(r"#\s*FIM ensaio=", bruto):
                fechar_registro()
            continue

        # ---- cabeçalho CSV
        if bruto.startswith("t_s,"):
            cabecalho = bruto.split(",")
            continue
        if cabecalho is None:
            continue

        campos = bruto.split(",")
        if len(campos) != len(cabecalho):
            continue  # linha cortada
        linha = dict(zip(cabecalho, campos))
        try:
            t = float(linha["t_s"])
            valores = (
                float(linha["v_bruta"]), float(linha["v_ema"]), float(linha["v_mm"]),
                float(linha["setpoint"]), float(linha["pulso"]), float(linha["dist_mm"]),
            )
        except ValueError:
            continue

        if arq_atual is None:
            abrir_registro("manual")
        if not cabecalho_escrito:
            arq_atual.write(",".join(cabecalho) + "\n")
            cabecalho_escrito = True
        arq_atual.write(bruto + "\n")

        dados["t"].append(t)
        for chave, v in zip(("bruta", "ema", "mm", "sp", "pulso", "dist"), valores):
            dados[chave].append(v)

        n += 1
        if n % 4 == 0:
            arq_atual.flush()
            redesenhar()

except KeyboardInterrupt:
    print("\nEncerrando a leitura...")
finally:
    ser.write(b"x\n")
    ser.close()
    fechar_registro()
    arquivo_eventos.close()
    print(f"Registros em {pasta}/. Gere os gráficos com:")
    print(f"  python scripts/gerar_graficos.py {pasta}/*.csv")
