# Relatório — Sensor TCRT5000 (Aula 13)

- `Relatorio_TCRT5000.pdf`: relatório compilado.
- `latex/`: fonte do relatório (`relatorio.tex`), figuras e cópia do sketch usada no apêndice.
  Compilar dentro de `latex/` com `pdflatex relatorio.tex` (duas vezes).
- `calibracao_tcrt5000.xlsx`: medidas, ajustes (g0, g1, R², RMSE) e tabela de resolução.
- `tcrt5000_distancia/src/main.cpp`: sketch que imprime `sinal,mm`.
- `plot_serial.py`: gráfico ao vivo da distância (`python3 plot_serial.py /dev/ttyACM0`).
