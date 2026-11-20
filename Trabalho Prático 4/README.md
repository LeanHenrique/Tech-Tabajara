# TP4 — Encoder e controle PID de velocidade da roda

Projeto de Sistemas Robóticos — DCC/UFMG — 2º semestre de 2026.

Bancada: Arduino Mega 2560, um lado do chassi impresso, um micro servo de giro contínuo e um encoder de quadratura.

Este TP fica na pasta `Trabalho Prático 4` do repositório da equipe: https://github.com/LeanHenrique/Tech-Tabajara/tree/main/Trabalho%20Pr%C3%A1tico%204

Os comandos abaixo valem a partir dessa pasta.

O relatório está em [`relatorio.pdf`](relatorio.pdf).

## Estrutura

```
src/main.cpp                  firmware (PlatformIO, Arduino Mega)
platformio.ini
fotos/                        fotos da montagem

```

## Ligações

| Sinal | Pino |
|---|---|
| Servo (sinal) | D9 |
| Encoder A | D2 (interrupção, pull-up interno) |
| Encoder B | D3 (interrupção, pull-up interno) |
| Botão (opcional, troca setpoint) | D4 (pull-up interno) |
| GND | comum entre Mega, servo e encoder |

## Parâmetros da equipe

| Grandeza | Valor | Como foi obtido |
|---|---|---|
| D (diâmetro na banda de contato) | 46,2 mm | volta marcada no papel (D = perímetro / π) |
| N (ticks por volta da roda, X4) | 288 | uma volta à mão, leitura do contador |
| k = πD/N | 0,5040 mm/tick (1,984 tick/mm) | calculado |
| Δt / degrau k/Δt | 50 ms / 10,1 mm/s | medido no código / calculado |
| Neutro | 1500 µs | valor usado no ensaio; busca pendente (comando `b`) |
| Correção máxima | ±250 µs em torno do neutro; pulso em 1000–2000 µs | enunciado (o ensaio do relatório usou ±500 µs) |
| kp | 0,1 µs/(mm/s) | sintonia na bancada |
| ki | 3,0 µs/mm | sintonia na bancada |
| kd | 0 µs·s²/mm | não usado (PI) |
| Filtro 1, usado no PID | passa-baixa de 1ª ordem, α = 0,20 | — |
| Filtro 2 | média móvel, janela = 5 amostras | implementado; comparação medida pendente (comando `g`) |

## Compilar e gravar

Com o PlatformIO (CLI ou extensão do VS Code), dentro desta pasta:

```
pio run              # compila
pio run -t upload    # grava na Mega
```

As bibliotecas Encoder (Paul Stoffregen) e Servo são instaladas pelo `platformio.ini`.

## Disparar os ensaios

```
pip install pyserial matplotlib pandas numpy
python scripts/plot_serial.py COM7          # troque pela porta da Mega
```

Os comandos são digitados no mesmo terminal. Cada ensaio gera um arquivo `registros/<ensaio>_<data>.csv` com estas colunas:
- `t_s`, `dt_ms`, `ticks`, `dist_mm`;
- `v_bruta`, `v_ema`, `v_mm`, `v_pid`;
- `setpoint`, `erro`, `P`, `I`, `D`, `pulso`;
- `modo`, `ensaio`, `fase`.

A linha `# cond`, no início do arquivo, registra neutro, k, Δt, ganhos e filtro.

| Comando | O que faz |
|---|---|
| `b` | busca do neutro: 1440 → 1560 µs em passos de 10 µs, 3 s cada (1 s descartado) |
| `a` | curva pulso × velocidade em malha aberta: 1000 → 2000 µs em passos de 20 µs, 2,5 s cada (1,5 s descartado) |
| `g 150` | degrau em malha aberta (neutro + 150 µs) para comparar os dois filtros no mesmo registro |
| `e` | ensaio 1: cinco patamares (20, 35, 50, 60, 70 mm/s; 6 s cada) |
| `i` | ensaio 2: inversão direta, +60 → −60 mm/s |
| `r` | ensaios 1 e 2 em sequência, no mesmo registro |
| `x` | para (pulso = neutro) |
| `v 40` / `p 1600` | setpoint manual (mm/s) / pulso fixo (µs) |
| `kp 0.2`, `ki 2`, `kd 0` | muda um ganho |
| `n 1510` | muda o neutro em tempo de execução |
| `f 0` / `f 1` | filtro do PID: 0 = passa-baixa, 1 = média móvel |
| `h` | imprime a condição atual |

Ordem recomendada na bancada:
1. Rode `b` e grave o neutro encontrado em `NEUTRO_MEDIDO` (`src/main.cpp`).
2. Rode `a` e ajuste os patamares de `ESCADA` e `RELATORIO` à curva medida.
3. Rode `g 150` e escolha o filtro.
4. Ajuste os ganhos.
5. Rode `e` e `i`.

Se a velocidade sair negativa com pulso acima do neutro, troque A e B (D2 ↔ D3) ou use `INVERTER_ENCODER = true`.

## Refazer os gráficos

```
python scripts/gerar_graficos.py registros/*.csv   # gráficos e tabelas de cada ensaio -> figuras/
```
