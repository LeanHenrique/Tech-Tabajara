#!/usr/bin/env python3
"""Plota ao vivo a serial do Arduino com a distância em milímetros.

    Cada linha chega como "sinal,mm". Desenha a segunda coluna.
    Linhas fora de alcance (mm negativo, inf ou nan) são ignoradas no gráfico.

    Uso: python3 plot_serial.py /dev/ttyACM0
    ou:  python3 plot_serial.py COM7
"""

import math
import sys
import time
from collections import deque

import matplotlib.pyplot as plt
import serial

porta = sys.argv[1] if len(sys.argv) > 1 else "COM7"
ser = serial.Serial(porta, 115200, timeout=1)
time.sleep(2)  # O Arduino reinicia ao abrir a conexão USB

n = 0
xs = deque(maxlen=200)
ys = deque(maxlen=200)

plt.ion()
fig, ax = plt.subplots()
(linha,) = ax.plot([], [], color="#960B22", lw=1.5)
ax.set_xlabel("Amostra")
ax.set_ylabel("Distância (mm)")
ax.set_title("TCRT5000: distância estimada")
fig.tight_layout()


def extrair_mm(linha_serial):
    """Retorna a distância em mm (segunda coluna) ou None se a linha não servir."""
    partes = linha_serial.split(",")
    if len(partes) != 2:
        return None
    try:
        mm = float(partes[1])
    except ValueError:
        return None
    if not math.isfinite(mm) or mm < 0:
        return None  # fora de alcance
    return mm


try:
    while True:
        bruto = ser.readline().decode(errors="ignore").strip()
        if not bruto:
            continue
        print(bruto, flush=True)  # registra todas as linhas, inclusive fora de alcance

        distancia_mm = extrair_mm(bruto)
        if distancia_mm is None:
            continue

        n += 1
        xs.append(n)
        ys.append(distancia_mm)
        linha.set_data(xs, ys)
        ax.relim()
        ax.autoscale_view()
        plt.pause(0.001)

except KeyboardInterrupt:
    print("\nEncerrando a leitura serial...")
finally:
    ser.close()
