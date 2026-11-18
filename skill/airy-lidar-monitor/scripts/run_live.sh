#!/usr/bin/env bash
# Live quality monitor for RoboSense Airy lidars, with a web dashboard.
#
#   bash run_live.sh --lidars <lidars.yaml> [options]
#
#   --lidars FILE      lidar list; multi_rslidar/config/lidars.yaml works as is
#                      (name, ip, msop_port, difop_port, imu_port per lidar)
#   --mode attach      (default) the robot's own drivers already publish /<name>/points;
#                      only listen. Nothing on the robot is started or changed.
#   --mode standalone  start one rslidar_sdk_node for all lidars (needs --ws)
#   --ws DIR           colcon workspace with rslidar_sdk built (sourced if given)
#   --duration D       90m (default), 2h, 1h30m, 300 (s), or forever
#   --bag              also record a bag of points + imu (~12 MB/s per lidar)
#   --capture C        auto (default) | sudo | docker | root | none
#                      raw packet capture for motor speed, lost packets, power, temperature
#   --docker-image I   image with python3 for --capture docker (default python:3-slim)
#   --iface IF         wired interface (default: the route to the first lidar)
#   --port P           dashboard port (default 8090, bound to 127.0.0.1)
#   --continue DIR     keep the dashboard timeline of an earlier run
#   --out DIR          where runs go (default ~/lidar_monitor)
#
# Dashboard: http://127.0.0.1:<port>/   Ctrl+C or `kill <pid>` stops cleanly.
# Output: <out>/runs/<stamp>/ live.csv (1 row/s), live.jsonl, wire.jsonl, events.jsonl, logs

HERE="$(cd "$(dirname "$0")" && pwd)"
LIDARS=""; MODE=attach; WS=""; DURATION=90m; BAGREC=0; CAPTURE=auto
IMAGE=python:3-slim; IFACE=""; PORT=8090; CONT=""; OUT="$HOME/lidar_monitor"
while [ $# -gt 0 ]; do
  case "$1" in
    --lidars) LIDARS="$2"; shift ;;
    --mode) MODE="$2"; shift ;;
    --ws) WS="$2"; shift ;;
    --duration) DURATION="$2"; shift ;;
    --bag) BAGREC=1 ;;
    --no-bag) BAGREC=0 ;;
    --capture) CAPTURE="$2"; shift ;;
    --docker-image) IMAGE="$2"; shift ;;
    --iface) IFACE="$2"; shift ;;
    --port) PORT="$2"; shift ;;
    --continue) CONT="$2"; shift ;;
    --out) OUT="$2"; shift ;;
    -h|--help) sed -n 2,26p "$0"; exit 0 ;;
    *) echo "unknown option $1 (see --help)"; exit 1 ;;
  esac; shift
done
RED='\033[0;31m'; GREEN='\033[0;32m'; CYAN='\033[0;36m'; YEL='\033[0;33m'; NC='\033[0m'
die() { echo -e "${RED}$*${NC}"; exit 1; }
[ -f "$LIDARS" ] || die "--lidars FILE is required (e.g. <ws>/multi_rslidar/config/lidars.yaml)"
LIDARS="$(readlink -f "$LIDARS")"

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
else SECS=$(to_secs "$DURATION") && [ -n "$SECS" ] || die "bad duration '$DURATION'"; fi
[ "$SECS" = 0 ] && [ "$BAGREC" = 1 ] && die "--duration forever cannot record a bag (it would fill the disk)"

# ---- ROS environment
if [ -z "$ROS_DISTRO" ]; then
  for f in /opt/ros/jazzy/setup.bash /opt/ros/humble/setup.bash /opt/ros/*/setup.bash; do
    [ -f "$f" ] && { source "$f"; break; }
  done
fi
[ -n "$WS" ] && source "$WS/install/setup.bash"
NOROS=()
if ! command -v ros2 >/dev/null; then
  [ "$MODE" = standalone ] && die "ROS 2 not found: standalone mode needs it (source it first)"
  [ "$BAGREC" = 1 ] && die "ROS 2 not found: --bag needs it (source it first)"
  echo -e "${YEL}ROS 2 not found: monitoring the cable and PC only (no point cloud / IMU topic values)${NC}"
  NOROS=(--no-ros)
fi

# ---- lidar list -> JSON (topics default to /<name>/points and /<name>/imu, as multi_rslidar)
STAMP=$(date +%Y%m%d_%H%M%S)
RUN="$OUT/runs/$STAMP"; mkdir -p "$RUN"
python3 - "$LIDARS" "$RUN/lidars.json" <<'EOF' || die "cannot read lidar list"
import json, sys, yaml
cfg = yaml.safe_load(open(sys.argv[1]))
out = []
for l in cfg['lidars']:
    n = l['name']
    imu = int(l.get('imu_port') or 0)
    out.append(dict(name=n, ip=l['ip'], msop_port=int(l['msop_port']), difop_port=int(l['difop_port']),
                    imu_port=imu, points_topic=l.get('points_topic', f'/{n}/points'),
                    imu_topic=l.get('imu_topic', f'/{n}/imu' if imu else '')))
ports = [p for l in out for p in (l['msop_port'], l['difop_port'], l['imu_port']) if p]
if len(ports) != len(set(ports)):
    sys.exit('two lidars share a port: every lidar needs its own msop/difop/imu ports')
json.dump(out, open(sys.argv[2], 'w'), indent=1)
for l in out:
    print(f"  {l['name']:14s} {l['ip']:16s} msop {l['msop_port']} difop {l['difop_port']} imu {l['imu_port']}  "
          f"-> {l['points_topic']} {l['imu_topic']}")
EOF
mapfile -t IPS < <(python3 -c "import json,sys; [print(l['ip']) for l in json.load(open(sys.argv[1]))]" "$RUN/lidars.json")
mapfile -t TOPICS < <(python3 -c "import json,sys; [print(t) for l in json.load(open(sys.argv[1])) for t in (l['points_topic'], l['imu_topic']) if t]" "$RUN/lidars.json")

# ---- checks
for ip in "${IPS[@]}"; do
  ping -c1 -W1 "$ip" >/dev/null 2>&1 || die "lidar $ip does not answer ping (cable, power, or PC address/route; see references/network-setup.md)"
done
[ -n "$IFACE" ] || IFACE=$(ip route get "${IPS[0]}" 2>/dev/null | grep -o 'dev [^ ]*' | cut -d' ' -f2)
[ -n "$IFACE" ] || die "cannot find the interface to ${IPS[0]}; pass --iface"
if ss -ltn "sport = :$PORT" | grep -q LISTEN; then die "port $PORT is busy (another monitor?) - use --port"; fi
if [ "$MODE" = standalone ]; then
  [ -n "$WS" ] || die "--mode standalone needs --ws <workspace with rslidar_sdk>"
  pgrep -x rslidar_sdk_nod >/dev/null && die "an rslidar_sdk_node already runs - use --mode attach, or stop it"
elif [ "$MODE" = attach ]; then
  if [ ${#NOROS[@]} = 0 ] && ! pgrep -x rslidar_sdk_nod >/dev/null; then echo -e "${YEL}no rslidar_sdk_node running: ROS values stay empty until the robot's lidar driver starts${NC}"; fi
else die "--mode must be attach or standalone"; fi
if [ "$BAGREC" = 1 ]; then
  FREE_GB=$(( $(df --output=avail -k "$OUT" | tail -1) / 1024 / 1024 ))
  NEED_GB=$(( SECS * 12 * ${#IPS[@]} / 1024 + 10 ))
  [ "$FREE_GB" -ge "$NEED_GB" ] || die "not enough disk for the bag: need ~${NEED_GB} GB, free ${FREE_GB} GB"
fi

# ---- raw capture method
if [ "$CAPTURE" = auto ]; then
  if [ "$(id -u)" = 0 ]; then CAPTURE=root
  elif sudo -n true 2>/dev/null; then CAPTURE=sudo
  elif command -v docker >/dev/null && docker info >/dev/null 2>&1; then CAPTURE=docker
  else CAPTURE=none; fi
fi
echo -e "${CYAN}mode $MODE, interface $IFACE, capture $CAPTURE, duration $([ "$SECS" = 0 ] && echo 'until stopped' || date -u -d @"$SECS" +%H:%M:%S), bag $([ $BAGREC = 1 ] && echo yes || echo no)${NC}"
[ "$CAPTURE" = none ] && echo -e "${YEL}no raw capture: motor speed, lost packets, power and temperature stay empty.${NC} Use sudo (sudo -v first) or docker."

echo "$$" > "$RUN/run.pid"
BAG="$OUT/bags/lidars_$STAMP"
CNAME="lidar_wire_$STAMP"
PIDS=(); WIREPID=""
cleanup() {
  trap - EXIT INT TERM
  touch "$RUN/STOP"                                   # wire_monitor exits on its own
  for _ in 1 2 3; do [ -n "$WIREPID" ] && kill -0 "$WIREPID" 2>/dev/null && sleep 1; done
  [ "$CAPTURE" = docker ] && docker rm -f "$CNAME" >/dev/null 2>&1
  [ "$CAPTURE" = sudo ] && sudo -n pkill -f "$RUN/[w]ire.jsonl" 2>/dev/null
  for p in "${PIDS[@]}"; do kill -TERM "$p" 2>/dev/null; done
  for _ in 1 2 3 4 5; do
    alive=0; for p in "${PIDS[@]}"; do kill -0 "$p" 2>/dev/null && alive=1; done
    [ "$alive" = 0 ] && break; sleep 1
  done
  for p in "${PIDS[@]}"; do kill -KILL "$p" 2>/dev/null; done   # rslidar_sdk_node ignores TERM
  wait 2>/dev/null
}
trap cleanup EXIT
trap 'cleanup; exit 143' TERM      # cleanup alone would return into the main loop

# ---- 1. raw packet monitor
WIRE_ARGS=("$IFACE" "$RUN/lidars.json" "$RUN/wire.jsonl" "$RUN/STOP")
case "$CAPTURE" in
  root) python3 "$HERE/wire_monitor.py" "${WIRE_ARGS[@]}" > "$RUN/wire.log" 2>&1 & WIREPID=$!; PIDS+=($!) ;;
  sudo) sudo -n python3 "$HERE/wire_monitor.py" "${WIRE_ARGS[@]}" > "$RUN/wire.log" 2>&1 & WIREPID=$! ;;
  docker) docker run --rm --name "$CNAME" --net=host --cap-add=NET_RAW -v "$HERE":/code:ro -v "$RUN":"$RUN" \
            "$IMAGE" python3 /code/wire_monitor.py "${WIRE_ARGS[@]}" > "$RUN/wire.log" 2>&1 & WIREPID=$! ;;
esac

# ---- 2. driver (standalone only)
if [ "$MODE" = standalone ]; then
  python3 - "$LIDARS" "$RUN/lidars.json" "$RUN/rslidar.yaml" <<'EOF'
import json, sys, yaml
cfg = yaml.safe_load(open(sys.argv[1])); lid = json.load(open(sys.argv[2]))
d0 = dict(lidar_type='RSAIRY', min_distance=0.1, max_distance=200, use_lidar_clock=False, dense_points=False,
          ts_first_point=True, start_angle=0, end_angle=360, user_layer_bytes=0, tail_layer_bytes=0)
d0.update(cfg.get('driver_defaults') or {})
out = dict(common=dict(msg_source=1, send_packet_ros=False, send_point_cloud_ros=True), lidar=[])
for l, raw in zip(lid, cfg['lidars']):
    d = dict(d0); d.update(raw.get('driver') or {})
    d.update(msop_port=l['msop_port'], difop_port=l['difop_port'], imu_port=l['imu_port'])
    n = l['name']
    out['lidar'].append(dict(driver=d, ros=dict(ros_frame_id=n, ros_recv_packet_topic=f'/{n}/packets',
        ros_send_packet_topic=f'/{n}/packets', ros_send_imu_data_topic=l['imu_topic'] or f'/{n}/imu',
        ros_send_point_cloud_topic=l['points_topic'], ros_queue_length=100)))
yaml.safe_dump(out, open(sys.argv[3], 'w'), sort_keys=False)
EOF
  NODE=$(find -L "$WS/install" -path '*lib/rslidar_sdk/rslidar_sdk_node' -type f 2>/dev/null | head -1)
  [ -x "$NODE" ] || die "rslidar_sdk_node not found under $WS/install"
  "$NODE" --ros-args -p config_path:="$RUN/rslidar.yaml" > "$RUN/driver.log" 2>&1 &
  PIDS+=($!)
fi

# ---- 3. monitor + dashboard
EXTRA=(); [ "$BAGREC" = 1 ] && EXTRA+=(--bag "$BAG"); [ -n "$CONT" ] && EXTRA+=(--continue "$CONT")
python3 "$HERE/live_monitor.py" --lidars "$RUN/lidars.json" --run "$RUN" --duration "$SECS" \
  --port "$PORT" --iface "$IFACE" "${NOROS[@]}" "${EXTRA[@]}" > "$RUN/monitor.log" 2>&1 &
MON=$!; PIDS+=($MON)
sleep 4
kill -0 "$MON" 2>/dev/null || { echo -e "${RED}monitor failed:${NC}"; tail -20 "$RUN/monitor.log"; exit 1; }

# ---- 4. bag
if [ "$BAGREC" = 1 ]; then
  mkdir -p "$OUT/bags"
  ros2 bag record -o "$BAG" -d 300 "${TOPICS[@]}" > "$RUN/record.log" 2>&1 &
  REC=$!; PIDS+=($REC)
fi

echo -e "${GREEN}dashboard: http://127.0.0.1:$PORT/${NC}   (remote: ssh -L $PORT:127.0.0.1:$PORT <robot>)"
echo "run files: $RUN"
[ "$BAGREC" = 1 ] && echo "bag:       $BAG"
echo "stop: Ctrl+C, or  kill $$"

STOP=0; trap 'STOP=1' INT
START=$(date +%s)
while :; do
  EL=$(( $(date +%s) - START ))
  [ "$SECS" -gt 0 ] && [ "$EL" -ge "$SECS" ] && break
  [ "$STOP" = 1 ] && { echo "stopping..."; break; }
  [ $((EL % 300)) -eq 0 ] && echo "  $(date -u -d @"$EL" +%H:%M:%S) elapsed, events: $(cat "$RUN/events.jsonl" 2>/dev/null | wc -l)"
  sleep 1
done
sleep 3     # let the dashboard see "finished"
if [ "$BAGREC" = 1 ]; then          # background ros2 bag ignores SIGINT; TERM closes it cleanly
  kill -TERM "$REC" 2>/dev/null
  for _ in $(seq 1 30); do kill -0 "$REC" 2>/dev/null || break; sleep 1; done
fi
trap - INT
cleanup
echo -e "${GREEN}done.${NC} files in $RUN"
[ "$BAGREC" = 1 ] && echo "bag: $BAG ($(du -sh "$BAG" 2>/dev/null | cut -f1))"
