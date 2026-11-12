# Airy packet layout (what the scripts read)

Every offset is from the start of the UDP payload, and every multi-byte number is big-endian unless noted. Source: `rs_driver/.../decoder_RSAIRY.hpp`, checked against live packets.

## Point packets (MSOP), 1,248 bytes, header id `55 aa 05 5a`, 2,240 /s at 600 rpm

| Offset | Size | Field | Use |
|---|---|---|---|
| 8 | 4 | `pkt_cnt_top2bot` | changes only every few packets; not used |
| 12 | 4 | `pkt_cnt_bot2top` | sequence counter, +1 per packet: lost = gap − 1; a jump backwards = reset/reboot |
| 20 | 6 + 4 | timestamp: seconds (6 bytes), microseconds (4 bytes) | lidar clock; the time between azimuth wraps = rotation period → measured rpm |
| 38–39 | 2 | main-board temperature | `t = ((b1 & 0x7F) << 5) + (b0 >> 3)`, negative if `b1 & 0x80`, × 0.0625 °C (rs_driver `parseTempInLe`) |
| 40–41 | 2 | top-board temperature | same encoding |
| 44 | 2 | azimuth of the first block | 0.01°. A drop of more than 180° = new turn |

## Status packets (DIFOP), 1,248 bytes, header id `a5 ff 00 5a`, 1 /s

| Offset | Size | Field | Scale (inferred) |
|---|---|---|---|
| 8 | 2 | `rpm` | set motor speed (600). Not a measurement |
| 1066 | 2 | `vol_main` | 0.01 V: input voltage (≈11.6–11.8 V on a 12 V supply) |
| 1068 | 2 | `vol_12v` | 0.01 V: internal 12 V rail |
| 1070 | 2 | `vol_mcu` | unknown scale (reads ~83) |
| 1072 | 2 | `vol_1v` | 0.01 V (reads ~1.01) |
| 1074 | 2 | `current_main` | 0.01 A (≈0.61) |

Offset 1066 is the `RSAIRYStatus` struct: 1248 − (tail 2 + reserved 110 + roll/yaw/pitch 12 + ptp_err 4 + xyz 12 + quaternion 16) − 26.

## IMU packets, header id `aa 55 5a 05`, 200 /s

- The last 6 bytes are the counter (4 bytes, big-endian, +1 per packet) and the tail (2 bytes, `f0 0f`).
- The driver publishes acceleration in **g**.

## Other frames

- **PTP:** 1 frame /s per lidar, Ethernet type 0x88f7. The PC counts these in `rx_dropped`; ignore it.
- **ARP:** a lidar keeps asking "who-has <host ip>" when the PC lacks that address, and sends nothing until answered.

## Driver facts

- `rslidar_sdk_node` ignores SIGTERM: stop it with SIGKILL after a grace period.
- Process name (`comm`) is truncated to `rslidar_sdk_nod`; use `pgrep -x rslidar_sdk_nod`.
- `use_lidar_clock: true` stamps with lidar time. Without PTP sync this differs from PC time, so delay values are meaningless. Gaps measured from stamps are still valid.
- The cloud is organised as width 96 (rings) × height 900 (columns): columns per scan = points / 96.
