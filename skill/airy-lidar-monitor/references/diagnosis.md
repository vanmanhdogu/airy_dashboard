# Diagnosing Airy problems

## Which layer: read the same seconds across all four

| What the dashboard shows | Layer | Next step |
|---|---|---|
| Motor speed below 600 rpm, or a "lidar restarted" event (packet counter started over after ~20 s of silence) | Lidar or its power: a restart means power was cut or dipped, or the lidar reset | Look at input voltage at the same seconds. A sag with the drop means power. Also check the main-board temperature |
| Input voltage drops more than 0.3 V from its starting value | Power supply / cable | Measure at the lidar connector under load. Other loads on the same supply (motors) may be the cause |
| No packets for seconds, the counter continues afterwards (big `msop_lost`, `msop_bad` 0), voltage steady, other lidars fine | Network path of that lidar: cable, connector, switch port | Swap cables/ports between lidars and see if the fault follows |
| Lost packets on the cable for several lidars at once | Shared switch, or the PC's network card | Check network-card overruns |
| UDP socket-buffer drops (`pc.udp_rcvbuf_err`) | PC: driver not reading fast enough | Check CPU and driver CPU. Raise `net.core.rmem_max` / the driver's socket buffer |
| Cable clean, no PC drops, but ROS rate < 10 Hz or gaps | Driver or ROS transport | Cross-check with a bag (`check_gaps.py`). If the bag is complete, it's the subscriber |
| IMU gaps on all lidars at the same moment, nothing on the cable | The process measuring them was busy (measurement artifact) | Gaps are measured from message stamps for this reason |

## Normal values (measured: 2 Airy units, standalone on a test bench)

| Metric | Normal |
|---|---|
| Point packets on the cable | 2,240 /s per lidar (224 per turn × 10) |
| IMU packets | 200 /s; status (DIFOP) packets 1 /s; PTP frames 1 /s |
| Motor speed (measured) | 600.0 rpm ±0.1; set value in DIFOP 600 |
| Point cloud rate in ROS | 10.00 Hz; longest gap ~100–110 ms; nothing over 150 ms in a 1 h run |
| Scan width | 900 columns. Each second, one scan of 899 and one of 861 is normal: about 36° near 322–358° missing once per second, firmware pattern, seen on both units |
| Points with a return | ~85–87 % indoors (depends on the scene). Ring 0 lowest. A fixed ~9° sector near 180° can have no returns (mount or cable) |
| Delay from scan start to ROS | ~102–107 ms (includes the 100 ms scan). Only meaningful with host-time stamps or PTP |
| Input voltage / current / power | 11.5–11.8 V, 0.61–0.62 A, 7.0–7.2 W per lidar, on a 12 V supply |
| Main board temperature | Rises from ~35 °C at power-on to ~85 °C in 30–60 min, then flat within ±0.5 °C |
| Top board temperature | ~20–23 °C, near room temperature, barely moves |
| Traffic | ~23 Mbit/s per lidar |
| Driver CPU | ~20–30 % of one core for 2 lidars |
| IMU | Acceleration is published in g, not m/s² (multiply by 9.81). Gyro bias of a few tenths of a °/s is normal |

## Known false alarms (already handled; keep them handled if editing)

- **Network card `rx_dropped` rises by 1 /s per lidar.** Linux counts frames of protocols nothing on the PC handles, here the lidars' PTP messages. Only `rx_fifo` (overruns) and `rx_errs` are real drops.
- **Missing scans that the bag does not show:**
  - A Python subscriber that deserializes 2.6 MB clouds falls behind.
  - A best-effort subscriber loses some large fragmented messages.
  - The monitor reads raw bytes with RELIABLE QoS. Robot programs that subscribe best-effort can lose clouds the same way: worth checking on the robot.
- **First seconds after start:** messages without stamps, start-up gaps. Events are suppressed for the first 10 s and charts skip the first 5 s.
- **Correctable PCIe errors in `dmesg`:** check which device. On the bench PC they came from the Wi-Fi card and were unrelated.

## What the power numbers can and cannot say

- **Source:** the lidar measures its own input voltage, 12 V rail and current. It reports them in its status packet once per second.
- **Scale:** 0.01 V / 0.01 A is inferred; RoboSense's driver code names the fields but never converts them. Confirm once with a multimeter, or change a bench-supply voltage and watch the value follow.
- **Current resolution:** 0.01 A, about ±0.06 W. Power differences between lidars under ~0.15 W mean nothing.
- **Speed:** too slow for millisecond dips. Use an oscilloscope or a fast logger for those.
- **Where it's measured:** inside the lidar, after the cable, so it includes cable losses.

## Field log

- **Bench, 2 Airy units, 2026-11-12.** One lidar went silent for 3 s (7,110 point packets lost):
  - its counter continued, so no restart
  - voltage steady
  - the other lidar on the same PC port was fine
  - verdict: that lidar's cable or switch port
- **Bench, same run, 10:44 and 10:46.** Lidar 1 restarted twice, 1.5 min apart. The restart signature, useful for recognising one:
  - data stops mid-second
  - ~10 s with no packets at all, not even status
  - status returns with ~0.27 A (booting, motor off)
  - current rises through 0.4–0.5 A (motor spin-up)
  - point data returns ~20 s after the stop, the packet counter starts from 0 (`msop_bad` = 1), and the first temperature reads 0

  The input voltage sagged 11.60 → 11.40 V in the 3 s before the first restart; it was steady before the second. Lidar 2, also on the bench, was unaffected. A full restart means the lidar lost power or reset. Ask whether someone handled its power connector before calling it a fault.
