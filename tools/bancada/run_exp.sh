#!/usr/bin/env bash
# usage: run_exp.sh <script_robot0> <script_robot1|none> [timeout_s]
# Roda um experimento do sumô na bancada (com janela), espera EXPERIMENT_DONE, encerra e copia o CSV.
S0=$1; S1=${2:-none}; TMO=${3:-240}
bash "$(dirname "$0")/bench_sync.sh" >/dev/null
cd ~/psr-hero-kit || exit 1
docker rm -f psrkit-ros >/dev/null 2>&1
LOG=$(mktemp)
echo "=== $S0 vs $S1 (timeout ${TMO}s)"
./run.sh run sumo "robot0:=$S0" "robot1:=$S1" > "$LOG" 2>&1 &
PID=$!
for _ in $(seq 1 100); do
  grep -q "process\[sumo_0" "$LOG" && break
  kill -0 $PID 2>/dev/null || break
  sleep 3
done
T0=$(date +%s)
while true; do
  grep -q "EXPERIMENT_DONE" "$LOG" && { echo "concluído em $(( $(date +%s) - T0 )) s"; break; }
  [ $(( $(date +%s) - T0 )) -ge "$TMO" ] && { echo "TIMEOUT (${TMO}s)"; break; }
  kill -0 $PID 2>/dev/null || { echo "run.sh terminou antes"; break; }
  sleep 3
done
sleep 2
docker rm -f psrkit-ros >/dev/null 2>&1
wait $PID 2>/dev/null
echo "--- erros no log: $(grep -ciE 'traceback|exception' "$LOG")"
grep -iE 'traceback|exception|erro no experimento|Error' "$LOG" | head -5 | cut -c1-200
grep -E "gravando|EXPERIMENT_DONE" "$LOG" | cut -c1-160
rm -f "$LOG"
DST="${TEAM:-/mnt/c/Users/marcu/OneDrive/Documentos/Robotica/Tech-Tabajara}/exercicios/registros/fase1
mkdir -p "$DST" && cp exercicios/registros/fase1/exp_*.csv "$DST"/ 2>/dev/null && echo "CSVs copiados: $(ls exercicios/registros/fase1 | tr '\n' ' ')"
