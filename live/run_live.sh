#!/usr/bin/env bash
# Live test of TWO RoboSense Airy lidars with a web dashboard.
#
#   bash run_live.sh [duration] [--no-bag] [--continue <run_dir>]
#     duration: 90m (default), 2h, 1h30m, 300 (s), or 'forever' (until stopped)
#     --continue: keep the dashboard timeline of an earlier run
#
# Opens: http://127.0.0.1:8090/   (refreshes every second)
# Output: ~/airy_test/live_runs/<stamp>/   live.csv (1 row/s), live.jsonl, wire.jsonl, events.jsonl, logs
#         ~/airy_test/bags/airy2x_<stamp>/ bag of /airyN/points + /airyN/imu (unless --no-bag)
# Ctrl+C (or `kill <pid>`) stops early; files are closed cleanly.

DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(dirname "$DIR")"
WS=/home/dogux/Downloads/iroi-a
IMAGE=ghcr.io/rsasaki0109/lidar_slam_ros2:jazzy      # only used for raw packet capture (needs NET_RAW)
IFACE=enp2s0
PORT=8090
DURATION=90m; BAGREC=1; CONT=""
while [ $# -gt 0 ]; do
  case "$1" in
    --no-bag) BAGREC=0 ;;
    --continue) CONT="$2"; shift ;;
    *) DURATION="$1" ;;
  esac; shift
done

RED='\033[0;31m'; GREEN='\033[0;32m'; CYAN='\033[0;36m'; NC='\033[0m'
to_secs() {
  local s="$1" total=0
  [[ "$s" =~ ^[0-9]+$ ]] && { echo "$s"; return; }
  [[ "$s" =~ ^([0-9]+h)?([0-9]+m)?([0-9]+s)?$ ]] || return 1
  [[ "$s" =~ ([0-9]+)h ]] && total=$((total + BASH_REMATCH[1] * 3600))
  [[ "$s" =~ ([0-9]+)m ]] && total=$((total + BASH_REMATCH[1] * 60))
  [[ "$s" =~ ([0-9]+)s ]] && total=$((total + BASH_REMATCH[1]))
  [ "$total" -gt 0 ] && echo "$total"
}
if [ "$DURATION" = forever ]; then SECS=0
else
  SECS=$(to_secs "$DURATION") || { echo -e "${RED}bad duration '$DURATION'${NC}"; exit 1; }
  [ -n "$SECS" ] || { echo -e "${RED}bad duration '$DURATION'${NC}"; exit 1; }
fi
[ "$SECS" = 0 ] && [ "$BAGREC" = 1 ] && { echo -e "${RED}'forever' needs --no-bag${NC} (a bag would fill the disk)"; exit 1; }

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

# ---- checks
for ip in 192.168.77.201 192.168.77.200; do
  ping -c1 -W1 "$ip" >/dev/null 2>&1 || {
    echo -e "${RED}lidar $ip does not answer.${NC} Set the wired port:"
    echo '  nmcli device modify enp2s0 ipv4.method manual ipv4.addresses 192.168.77.191/32 \'
    echo '        ipv4.routes "192.168.77.201/32, 192.168.77.200/32" ipv4.gateway "" ipv4.never-default yes'
    exit 1; }
done
if pgrep -x rslidar_sdk_nod >/dev/null; then
  echo -e "${RED}an rslidar_sdk_node is already running${NC} (PID $(pgrep -x rslidar_sdk_nod | tr '\n' ' ')). Stop it first."; exit 1
fi
if ss -ltn "sport = :$PORT" | grep -q LISTEN; then
  echo -e "${RED}port $PORT is busy${NC} (another monitor running?)"; exit 1
fi
mkdir -p "$ROOT/bags" "$ROOT/live_runs"
FREE_GB=$(( $(df --output=avail -k "$ROOT/bags" | tail -1) / 1024 / 1024 ))
NEED_GB=$(( BAGREC * SECS * 24 / 1024 + 10 ))
echo -e "${CYAN}duration $([ "$SECS" = 0 ] && echo "until stopped" || date -u -d @"$SECS" +%H:%M:%S), bag ~$(( BAGREC * SECS * 24 / 1024 )) GB, free ${FREE_GB} GB${NC}"
[ "$FREE_GB" -ge "$NEED_GB" ] || { echo -e "${RED}not enough disk${NC} (need ~${NEED_GB} GB) - use --no-bag or a shorter run"; exit 1; }

STAMP=$(date +%Y%m%d_%H%M%S)
RUN="$ROOT/live_runs/$STAMP"; mkdir -p "$RUN"
BAG="$ROOT/bags/airy2x_$STAMP"
CNAME="airy_wire_$STAMP"
echo "$$" > "$RUN/run.pid"

PIDS=()
cleanup() {
  trap - EXIT INT TERM
  docker rm -f "$CNAME" >/dev/null 2>&1
  for p in "${PIDS[@]}"; do kill -TERM "$p" 2>/dev/null; done
  for _ in 1 2 3 4 5; do
    alive=0; for p in "${PIDS[@]}"; do kill -0 "$p" 2>/dev/null && alive=1; done
    [ "$alive" = 0 ] && break; sleep 1
  done
  for p in "${PIDS[@]}"; do kill -KILL "$p" 2>/dev/null; done
  wait 2>/dev/null
}
trap cleanup EXIT
trap 'cleanup; exit 143' TERM      # cleanup alone would return into the main loop

# ---- 1. raw packet monitor (container: needs NET_RAW, no sudo)
docker run --rm --name "$CNAME" --net=host --cap-add=NET_RAW \
  -v "$DIR":/code:ro -v "$RUN":/out "$IMAGE" python3 /code/wire_monitor.py "$IFACE" /out/wire.jsonl \
  > "$RUN/wire.log" 2>&1 &
PIDS+=($!)

# ---- 2. driver: both lidars in one node
"$WS/install/rslidar_sdk/lib/rslidar_sdk/rslidar_sdk_node" --ros-args -p config_path:="$DIR/airy2.yaml" \
  > "$RUN/driver.log" 2>&1 &
PIDS+=($!)

# ---- 3. monitor + dashboard
BAGARG=(); [ "$BAGREC" = 1 ] && BAGARG=(--bag "$BAG")
CONTARG=(); [ -n "$CONT" ] && CONTARG=(--continue "$CONT")
python3 "$DIR/live_monitor.py" --run "$RUN" --duration "$SECS" --port "$PORT" "${BAGARG[@]}" "${CONTARG[@]}" \
  > "$RUN/monitor.log" 2>&1 &
MON=$!; PIDS+=($MON)
sleep 4
kill -0 "$MON" 2>/dev/null || { echo -e "${RED}monitor failed${NC}:"; tail -20 "$RUN/monitor.log"; exit 1; }

# ---- 4. bag
if [ "$BAGREC" = 1 ]; then
  ros2 bag record -o "$BAG" -d 300 /airy1/points /airy1/imu /airy2/points /airy2/imu \
    > "$RUN/record.log" 2>&1 &
  REC=$!; PIDS+=($REC)
fi

echo -e "${GREEN}dashboard: http://127.0.0.1:$PORT/${NC}"
echo "run files: $RUN"
[ "$BAGREC" = 1 ] && echo "bag:       $BAG"
echo "stop early: Ctrl+C, or  kill $(cat "$RUN/run.pid")"

STOP=0; trap 'STOP=1' INT
START=$(date +%s)
while :; do
  EL=$(( $(date +%s) - START ))
  [ "$SECS" -gt 0 ] && [ "$EL" -ge "$SECS" ] && break
  [ "$STOP" = 1 ] && { echo "stopping early..."; break; }
  if [ $((EL % 300)) -eq 0 ]; then
    echo "  $(date -u -d @"$EL" +%H:%M:%S) elapsed  events: $(cat "$RUN/events.jsonl" 2>/dev/null | wc -l)"
  fi
  sleep 1
done
sleep 3     # let the dashboard see "finished"
if [ "$BAGREC" = 1 ]; then
  kill -TERM "$REC" 2>/dev/null
  for _ in $(seq 1 30); do kill -0 "$REC" 2>/dev/null || break; sleep 1; done
fi
trap - INT
cleanup
echo -e "${GREEN}done.${NC} files in $RUN"
[ "$BAGREC" = 1 ] && echo "bag: $BAG ($(du -sh "$BAG" 2>/dev/null | cut -f1))"
