#!/bin/sh
# Usage: realrun.sh <label> <seconds> [enable]
# Streams robot.state at 50 Hz straight to this machine (so a robot that reboots on a fall
# still leaves everything up to the fall here), optionally handing the robot to its policy
# one second in and taking it back at the end.
set -u
L=$1; T=$2; E=${3:-}
OUT="$(dirname "$0")/runs/$L.jsonl"
ssh -o BatchMode=yes -o ServerAliveInterval=2 -o ServerAliveCountMax=2 microduck "timeout $T robotctl monitor --json --hz 50" > "$OUT" 2>/dev/null &
REC=$!
sleep 1.5
if [ "$E" = enable ]; then ssh -o BatchMode=yes -o ConnectTimeout=5 microduck 'timeout 5 robotctl robot enable' >/dev/null 2>&1 & fi
wait $REC
if [ "$E" = enable ]; then ssh -o BatchMode=yes -o ConnectTimeout=5 microduck 'timeout 5 robotctl robot enable --off' >/dev/null 2>&1; fi
echo "$(wc -l < "$OUT") frames -> $OUT"
