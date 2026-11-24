#!/usr/bin/env bash
# usage: watch_sumo.sh <secs> <args do run.sh sumo ...>
SECS=$1; shift
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
docker exec psrkit-ros /ros-entrypoint.sh bash -c "python3 /ws/psr_exercicios/tools/watch_sumo.py $SECS 0.4" 2>&1 | cut -c1-200
docker rm -f psrkit-ros >/dev/null 2>&1
wait $PID 2>/dev/null
rm -f "$LOG"
