#!/usr/bin/env bash
# usage: watch_ctf.sh <secs> <args do run.sh ctf ...>   ex.: watch_ctf.sh 50 blue:=none
SECS=$1; shift
bash "$(dirname "$0")/bench_sync.sh" >/dev/null
cd ~/psr-hero-kit || exit 1
docker rm -f psrkit-ros >/dev/null 2>&1
LOG=$(mktemp)
./run.sh run ctf "$@" > "$LOG" 2>&1 &
PID=$!
for _ in $(seq 1 100); do
  grep -q "process\[ctf" "$LOG" && break
  kill -0 $PID 2>/dev/null || break
  sleep 3
done
sleep 3
docker exec psrkit-ros /ros-entrypoint.sh bash -c "python3 /ws/psr_exercicios/tools/watch_ctf.py $SECS 1.0" 2>&1 | cut -c1-260
echo "--- transições de estado do nosso nó:"
grep -E "estado:|falhou|Traceback" "$LOG" | tail -n 25 | cut -c1-140
docker rm -f psrkit-ros >/dev/null 2>&1
wait $PID 2>/dev/null
rm -f "$LOG"
