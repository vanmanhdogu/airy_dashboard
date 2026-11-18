---
name: airy-lidar-monitor
description: Check, monitor and diagnose RoboSense Airy 3D lidars on a PC or robot running ROS 2. Finds lidars on the cable (IP, MAC, ports, port clashes), runs a live web dashboard of frequency, motor speed, voltage/current/power, temperatures, lost packets, gaps, IMU rate and PC load, logs events, and works out whether a problem comes from the lidar, its power, the network, the PC or the ROS subscriber. Use it whenever someone wants to check an Airy or rslidar connection, test or compare lidar quality, monitor lidars in real time or over hours, or find out why lidars on a robot drop frames, run below 10 Hz, freeze, lose points or "work badly", even if they don't say "Airy" or "dashboard".
---

# Airy lidar monitor

Everything here was built and verified on real Airy lidars (rslidar_sdk driver, ROS 2 Jazzy).
The scripts are in `scripts/`; read the reference files when the step points to them.

## Why it is built this way

A lidar problem can come from four layers, and each needs its own measurement:

| Layer | Measured from | Tells you |
|---|---|---|
| Lidar + power | raw packets on the cable (`wire_monitor.py`) | real motor speed from the lidar's own timestamps; voltage, current, power and temperatures from its status packets |
| Network | the lidar's packet sequence counters | packets lost between lidar and PC |
| PC | `/proc` | UDP socket-buffer drops, network-card overruns, CPU, driver CPU |
| ROS | subscribing to the point/IMU topics | rate, longest gap, valid points, delay: what a robot program actually gets |

Comparing the layers second by second is what makes a diagnosis possible. For example,
motor speed stays at 600 rpm while the ROS rate drops: the cause is after the lidar.
`references/diagnosis.md` has the full table, the normal values and the known false alarms.

## Workflow

### 1. Find the lidars and check the network

Run discovery first. It needs raw capture (root, sudo or docker) but no IP set-up and no driver:

```bash
sudo python3 scripts/discover.py <iface> 6
# or, without sudo:
docker run --rm --net=host --cap-add=NET_RAW -v "$PWD/scripts":/code:ro python:3-slim python3 /code/discover.py <iface> 6
```

Look for:
- **No data, but the lidar asks "who-has <ip>":** the PC lacks the host address the lidar sends to.
- **Two lidars on the same port:** the driver can't separate them. Each lidar needs its own MSOP, DIFOP and IMU ports, set in its web page.
- **Wrong destination IP.**

Fixes: `references/network-setup.md`. Don't change a lidar's web settings yourself; tell the user what to change.
Ping each lidar before going on.

### 2. Get the lidar list

The monitor reads a YAML file with one entry per lidar: `name`, `ip`, `msop_port`, `difop_port`, `imu_port` (0 = no IMU).
- **On the iROI-A robot,** use `multi_rslidar/config/lidars.yaml` as is.
- **Elsewhere,** copy `scripts/lidars.example.yaml`.

Topics default to `/<name>/points` and `/<name>/imu`. Override them with `points_topic` / `imu_topic`.

### 3. Choose the mode

- **attach** (default; use it on a robot): the robot's own drivers keep running. The monitor only listens:
  - raw capture on the cable is passive
  - it adds one ROS subscriber per topic, about 35 % of one core for 2 lidars
- **standalone** (bench test, nothing else running): starts one `rslidar_sdk_node` for all lidars. It needs `--ws <workspace with rslidar_sdk>`.

  Never use standalone while another rslidar driver runs. Both sockets can bind the same port, but only one receives the packets, so the robot's driver would go blind. The script refuses to start in that case.

### 4. Start the run, detached so it survives the session

```bash
cd <skill>/scripts
setsid nohup bash run_live.sh --lidars <lidars.yaml> --duration 90m \
  > ~/lidar_monitor/console_$(date +%H%M%S).log 2>&1 < /dev/null &
```

Options:
- `--duration`: `forever` runs until stopped.
- `--bag`: also records points and IMU, ~12 MB/s per lidar. Check the disk; `forever` refuses a bag.
- `--capture sudo|docker|root|none`: `auto` picks root, then passwordless sudo, then docker.
  - With sudo, ask the user to run `sudo -v` first.
  - Docker needs an image with python3: `--docker-image`.
  - With `none`, the motor speed, power, lost-packet and temperature cards stay empty.
- `--port 8090`: the dashboard address is http://127.0.0.1:8090/.
- `--continue <run_dir>`: keeps the timeline of an earlier run, e.g. switching from a fixed-length run with a bag to `forever`.

The dashboard has **Standby / Run** buttons per lidar and for all lidars. They set the lidar's own "Operation Mode" through its web page (`scripts/airy_web.py`, also usable on its own: `python3 airy_web.py <ip> [standby|run]`). In standby the motor and laser are off. The lidar's problem checks pause while it is in standby and for 30 s after Run. These buttons are for the user to press; don't use them yourself unless asked. `live_monitor.py --control off` hides them; `--control dry-run` shows them but sends nothing to the lidars.

Don't use `timeout` on `docker run`: the signal doesn't reach the container. The scripts stop their own container with a stop file.

Then check that data is flowing before telling the user it works:

```bash
curl -s 'http://127.0.0.1:8090/data?from=0' | python3 -c "import sys,json; j=json.load(sys.stdin); print(j['total'], j['rows'][-1])"
```

`rows[-1]` should show:
- `pts_hz` ≈ 10
- `rpm_meas` ≈ 600
- `power` filled in
- `pc.drivers` ≥ 1

If ROS values are empty in attach mode, the robot's driver is not running, or the shell has a different `ROS_DOMAIN_ID` or RMW than the robot. Run the script from the same environment as the robot's nodes.

Viewing from another PC: `ssh -L 8090:127.0.0.1:8090 <robot>`, then open http://127.0.0.1:8090/ locally.

### 5. Read the results

- **Files:** `<out>/runs/<stamp>/` holds
  - `live.csv`: one row per second, every metric, `<lidar>_<metric>` columns
  - `events.jsonl`: start and end of every out-of-range condition
  - `wire.jsonl`: raw cable data
- **Workflow:** read the events first. For each one, check the other layers at the same seconds in `live.jsonl` / `wire.jsonl` and apply `references/diagnosis.md`.
- **Be careful with conclusions.** For example, when one lidar's data stops for seconds, check its packet counter (`msop_lost` large, `msop_bad` 0):
  - counter kept counting: the lidar didn't restart, so it's the network path
  - steady voltage: not a power dip, but only at 1-second resolution

### 6. Make sure the monitor is not the problem

If the ROS side reports gaps, compare against an independent subscriber before blaming the lidar. With `--bag`, the first split file closes after 5 minutes:

```bash
python3 scripts/check_gaps.py <run_dir> <bag>/<bag>_0.mcap
```

Bag complete but monitor shows gaps: the monitor missed messages, not the driver. The ROS subscriber is already written to avoid the two causes found so far: Python deserialization was too slow, and best-effort QoS lost large clouds. Keep it that way if you edit it.

### 7. Stop

`kill <pid in <run>/run.pid>` (TERM). The script then:
- closes the bag cleanly
- stops the capture
- kills the driver, which ignores TERM

Don't use `pkill -f <pattern>` from a shell whose own command line contains the pattern: it kills that shell.

## Reporting to the user

Give:
- a per-lidar table of the key values (rate, motor speed, power, voltage, current, temperatures, lost packets)
- every event, with which layer it points to and why
- what is measured versus inferred: the voltage/current scale, the 1-second power resolution

Unexplained drops on a robot, while this monitor sees every scan, point to the robot's own subscribers. Lidar programs often subscribe best-effort, and large clouds then get lost. Say so.

## Files

- `scripts/run_live.sh`: starts and stops everything (`--help`)
- `scripts/wire_monitor.py`: raw cable measurements (stdlib only, runs in any python3 container)
- `scripts/live_monitor.py`: ROS + PC side, CSV/JSON logs, serves the dashboard
- `scripts/dashboard.html`: live page, one card per lidar (up to 4), charts, events
- `scripts/discover.py`: find lidars, ports and clashes on a cable
- `scripts/check_gaps.py`: bag vs monitor cross-check
- `scripts/lidars.example.yaml`: config template
- `references/diagnosis.md`: layer table, normal values, false alarms, field findings
- `references/airy-packets.md`: byte offsets of every value read from the packets
- `references/network-setup.md`: host address, routes, ports, web UI
