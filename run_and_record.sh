#!/usr/bin/env bash
# Run ONE RoboSense Airy in ROS 2, record a bag, then report topic Hz + temperature.
#
#   bash run_and_record.sh [duration] [--packets]
#     duration   30 (seconds), 90m, 2h, 1h30m ...   (default 30s)
#     --packets  also record /rslidar_packets (raw, ~3 MB/s more)
#
# Ctrl+C stops the recording early; the bag and report are still produced.
# For long runs start it inside tmux, so closing the terminal does not stop it.
#
# Output: ~/airy_test/bags/airy_<date>_<time>/   bag, split into 5-minute files
#         ~/airy_test/logs/temperature_<date>_<time>.csv   1 Hz temperatures
#         ~/airy_test/logs/*_<date>_<time>.log            driver / recorder logs

DIR="$(cd "$(dirname "$0")" && pwd)"
WS=/home/dogux/Downloads/iroi-a          # where rslidar_sdk is built
LIDAR_IP=192.168.77.201
SPLIT_SECS=300                           # one bag file per 5 minutes
MB_PER_SEC=12                            # measured ~11 MB/s (points + imu)
DURATION=30s; PACKETS=0
for a in "$@"; do
  case "$a" in
    --packets) PACKETS=1 ;;
    *) DURATION="$a" ;;
  esac
done

GREEN='\033[0;32m'; RED='\033[0;31m'; CYAN='\033[0;36m'; NC='\033[0m'

# duration -> seconds: 45 | 45s | 90m | 2h | 1h30m
to_secs() {
  local s="$1" total=0
  [[ "$s" =~ ^[0-9]+$ ]] && { echo "$s"; return; }
  [[ "$s" =~ ^([0-9]+h)?([0-9]+m)?([0-9]+s)?$ ]] || return 1
  [[ "$s" =~ ([0-9]+)h ]] && total=$((total + BASH_REMATCH[1] * 3600))
  [[ "$s" =~ ([0-9]+)m ]] && total=$((total + BASH_REMATCH[1] * 60))
  [[ "$s" =~ ([0-9]+)s ]] && total=$((total + BASH_REMATCH[1]))
  [ "$total" -gt 0 ] && echo "$total"
}
SECS=$(to_secs "$DURATION") || { echo -e "${RED}bad duration '$DURATION'${NC} (use 30, 90m, 2h, 1h30m)"; exit 1; }
[ -n "$SECS" ] || { echo -e "${RED}bad duration '$DURATION'${NC}"; exit 1; }
[ "$PACKETS" = 1 ] && MB_PER_SEC=$((MB_PER_SEC + 3))

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

# 1. checks: lidar reachable, no driver already running, enough disk
if ! ping -c1 -W1 "$LIDAR_IP" >/dev/null 2>&1; then
  echo -e "${RED}lidar $LIDAR_IP does not answer.${NC} Set the wired port first:"
  echo '  nmcli device modify enp2s0 ipv4.method manual ipv4.addresses 192.168.77.191/32 \'
  echo '        ipv4.routes 192.168.77.201/32 ipv4.gateway "" ipv4.never-default yes'
  exit 1
fi
if pgrep -x rslidar_sdk_nod >/dev/null; then
  echo -e "${RED}an rslidar_sdk_node is already running${NC} (PID $(pgrep -x rslidar_sdk_nod | tr '\n' ' '))."
  echo "  Only one process can receive the lidar packets. Stop it first:  pkill -KILL -x rslidar_sdk_nod"
  exit 1
fi
mkdir -p "$DIR/bags" "$DIR/logs"
NEED_GB=$(( SECS * MB_PER_SEC / 1024 + 10 ))        # + 10 GB margin
FREE_GB=$(( $(df --output=avail -k "$DIR/bags" | tail -1) / 1024 / 1024 ))
echo -e "${CYAN}duration $(date -u -d @"$SECS" +%H:%M:%S), estimated bag ~$(( SECS * MB_PER_SEC / 1024 )) GB, free ${FREE_GB} GB${NC}"
if [ "$FREE_GB" -lt "$NEED_GB" ]; then
  echo -e "${RED}not enough disk space${NC} (need ~${NEED_GB} GB incl. 10 GB margin)"; exit 1
fi

PIDS=()
# Background jobs of a non-interactive shell ignore SIGINT, and the driver also
# ignores TERM: stop with TERM, then KILL whatever is left after 5 s.
cleanup() {
  trap - EXIT INT TERM
  for p in "${PIDS[@]}"; do kill -TERM "$p" 2>/dev/null; done
  for _ in 1 2 3 4 5; do
    alive=0; for p in "${PIDS[@]}"; do kill -0 "$p" 2>/dev/null && alive=1; done
    [ "$alive" = 0 ] && return; sleep 1
  done
  for p in "${PIDS[@]}"; do kill -KILL "$p" 2>/dev/null; done
  wait 2>/dev/null
}
trap cleanup EXIT TERM

STAMP=$(date +%Y%m%d_%H%M%S)
CSV="$DIR/logs/temperature_$STAMP.csv"

# 2. driver (binary itself, so $! is the node PID) + temperature publisher
echo -e "${CYAN}starting driver...${NC}"
"$WS/install/rslidar_sdk/lib/rslidar_sdk/rslidar_sdk_node" --ros-args -p config_path:="$DIR/airy.yaml" \
  > "$DIR/logs/driver_$STAMP.log" 2>&1 &
PIDS+=($!)
python3 "$DIR/airy_temperature.py" --ros-args -p csv:="$CSV" \
  > "$DIR/logs/temperature_$STAMP.log" 2>&1 &
PIDS+=($!)
sleep 4

# 3. record in the background, print progress every minute
TOPICS=(/rslidar_points /rslidar_imu_data /airy/temperature /airy/temperature_topboard)
[ "$PACKETS" = 1 ] && TOPICS+=(/rslidar_packets)
BAG="$DIR/bags/airy_$STAMP"
echo -e "${CYAN}recording -> $BAG${NC}  (Ctrl+C to stop early)"
echo "  topics: ${TOPICS[*]}"
ros2 bag record -o "$BAG" -d "$SPLIT_SECS" "${TOPICS[@]}" > "$DIR/logs/record_$STAMP.log" 2>&1 &
REC=$!
PIDS+=($REC)

STOP=0
trap 'STOP=1' INT
START=$(date +%s)
while kill -0 "$REC" 2>/dev/null; do
  ELAPSED=$(( $(date +%s) - START ))
  [ "$ELAPSED" -ge "$SECS" ] && break
  [ "$STOP" = 1 ] && { echo -e "\n${CYAN}stopping early...${NC}"; break; }
  if [ $((ELAPSED % 60)) -eq 0 ]; then
    T=$(tail -1 "$CSV" 2>/dev/null | awk -F, 'NR==1 && $2+0==$2 {printf "main %.2f C  top %.2f C", $2, $3}')
    printf '  %s / %s   bag %s   %s\n' "$(date -u -d @"$ELAPSED" +%H:%M:%S)" \
      "$(date -u -d @"$SECS" +%H:%M:%S)" "$(du -sh "$BAG" 2>/dev/null | cut -f1)" "${T:-temp n/a}"
  fi
  sleep 1
done
if ! kill -0 "$REC" 2>/dev/null; then
  echo -e "${RED}ros2 bag record exited early${NC} - see $DIR/logs/record_$STAMP.log"
fi
# Stop the recorder: as a background job it ignores SIGINT, but rosbag2 also
# closes the bag cleanly (metadata.yaml written) on SIGTERM.
kill -TERM "$REC" 2>/dev/null
for _ in $(seq 1 30); do kill -0 "$REC" 2>/dev/null || break; sleep 1; done
trap - INT
cleanup

# 4. report
echo
python3 "$DIR/bag_report.py" "$BAG" 2>&1 | grep -v WARN
echo -e "\n${GREEN}bag size: $(du -sh "$BAG" | cut -f1)   files: $(ls "$BAG"/*.mcap 2>/dev/null | wc -l)${NC}"
echo "temperature CSV: $CSV"
