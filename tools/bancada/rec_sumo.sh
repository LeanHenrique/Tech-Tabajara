#!/usr/bin/env bash
# usage: rec_sumo.sh <nome_do_registro> <secs> <periodo> <args do run.sh sumo ...>
# Grava as poses dos dois robôs do sumô (watch_sumo.py) em um .txt e copia para o repositório.
NOME=$1; SECS=$2; PER=$3; shift 3
bash "$(dirname "$0")/bench_sync.sh" >/dev/null
cd ~/psr-hero-kit || exit 1
docker rm -f psrkit-ros >/dev/null 2>&1
LOG=$(mktemp)
./run.sh run sumo "$@" > "$LOG" 2>&1 &
PID=$!
for _ in $(seq 1 100); do
  grep -q "process\[sumo" "$LOG" && break
  kill -0 $PID 2>/dev/null || break
  sleep 3
done
sleep 3
mkdir -p exercicios/registros/fase4
docker exec psrkit-ros /ros-entrypoint.sh bash -c "python3 /ws/psr_exercicios/tools/watch_sumo.py $SECS $PER" > "$HOME/rec_$NOME.txt" 2>&1
docker rm -f psrkit-ros >/dev/null 2>&1
wait $PID 2>/dev/null
rm -f "$LOG"
DST="${TEAM:-/mnt/c/Users/marcu/OneDrive/Documentos/Robotica/Tech-Tabajara}/exercicios/registros/fase4
mkdir -p "$DST" && cp "$HOME/rec_$NOME.txt" "$DST/$NOME.txt" && echo "gravado: $NOME.txt ($(grep -c '^rt=' "$HOME/rec_$NOME.txt") linhas)"
