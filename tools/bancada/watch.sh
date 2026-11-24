#!/usr/bin/env bash
# usage: watch.sh <secs> <args do run.sh soccer ...>   ex.: watch.sh 60 blue:=none
SECS=$1; shift
bash "$(dirname "$0")/bench_sync.sh" >/dev/null
cd ~/psr-hero-kit || exit 1
docker rm -f psrkit-ros >/dev/null 2>&1
LOG=$(mktemp)
./run.sh run soccer "$@" > "$LOG" 2>&1 &
PID=$!
for _ in $(seq 1 100); do
  grep -q "process\[soccer" "$LOG" && break
  kill -0 $PID 2>/dev/null || break
  sleep 3
done
sleep 4
docker exec psrkit-ros /ros-entrypoint.sh bash -c "python3 /ws/psr_exercicios/tools/watch_soccer.py $SECS 2" 2>&1 | cut -c1-260
docker rm -f psrkit-ros >/dev/null 2>&1
wait $PID 2>/dev/null
rm -f "$LOG"
