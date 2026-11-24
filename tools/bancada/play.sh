#!/usr/bin/env bash
# usage: play.sh <sumo|ctf|soccer> <secs> <label> [args do run.sh ...]
# Roda uma partida de teste com o simulador (e janela), lê o placar (/game/state) no começo e no fim e
# procura traceback/exceção no log. Ex.: play.sh sumo 120 "hero_0 x parado" robot1:=none
GAME=$1; SECS=$2; LABEL=$3; shift 3
bash "$(dirname "$0")/bench_sync.sh" >/dev/null
cd ~/psr-hero-kit || exit 1
docker rm -f psrkit-ros >/dev/null 2>&1
# FRESH=1: reinicia o simulador (robôs e bola nas posições de nascimento): evita contaminação entre partidas
[ "${FRESH:-0}" = 1 ] && docker rm -f psrkit-sim >/dev/null 2>&1
LOG=$(mktemp)
echo "=== $GAME: $LABEL   (./run.sh run $GAME $*)"
./run.sh run "$GAME" "$@" > "$LOG" 2>&1 &
PID=$!
for _ in $(seq 1 100); do
  grep -q "process\[${GAME}" "$LOG" && break
  kill -0 $PID 2>/dev/null || break
  sleep 3
done
estado() { docker exec psrkit-ros /ros-entrypoint.sh bash -c "timeout 6 rostopic echo -n 1 -p /game/state/data" 2>/dev/null | tail -n 1 | cut -c1-230; }
sleep 8
echo "  início : $(estado)"
sleep "$SECS"
echo "  fim    : $(estado)"
docker rm -f psrkit-ros >/dev/null 2>&1
wait $PID 2>/dev/null
echo "  traceback/exceção: $(grep -ciE 'traceback|exception' "$LOG") | 'falhou': $(grep -c 'falhou' "$LOG") | transições de estado: $(grep -c 'estado:' "$LOG")"
grep -iE 'traceback|exception|falhou' "$LOG" | head -3 | cut -c1-180
grep 'estado:' "$LOG" | head -4 | cut -c1-120
rm -f "$LOG"
