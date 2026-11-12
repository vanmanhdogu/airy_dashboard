# Network set-up for Airy lidars

Each lidar sends UDP to one **destination (host) IP**, set in its web page (http://<lidar ip>/). The PC must own that address on the wired port.

## PC address without breaking other networks

If the lidar subnet overlaps another interface (e.g. Wi-Fi on the same /24), give the wired port a /32 address plus one route per lidar:

```bash
nmcli device modify <iface> ipv4.method manual ipv4.addresses <host_ip>/32 \
  ipv4.routes "<lidar1_ip>/32, <lidar2_ip>/32" ipv4.gateway "" ipv4.never-default yes
```

- **`device modify` is temporary:** it's lost when the cable is unplugged or the PC reboots. That's the most common reason "the lidar stopped working". Use `nmcli connection modify <con> ...` for a permanent setting, with the user's agreement.
- **Without a clash,** a normal /24 address works: `ipv4.addresses <host_ip>/24`.
- **Symptoms of a missing address:**
  - `ip -br addr` shows no IPv4 on the port
  - NetworkManager says "connecting (getting IP configuration)"
  - discovery shows the lidar asking ARP "who-has <host ip>"

## Ports: different for every lidar

All lidars send to the same host IP. Only one socket receives each port, and the driver can only filter by port, not by source IP. So each lidar needs its own three ports, set in its web page:

| Lidar | MSOP (points) | DIFOP (status) | IMU |
|---|---|---|---|
| 1 | 6699 | 7788 | 6688 |
| 2 | 6700 | 7789 | 6689 |
| 3 | 6701 | 7790 | 6690 |

The driver config (`lidars.yaml`) must match the web pages. After changing ports, check them with `discover.py`.

## Known address plans

- **iROI-A robot** (`multi_rslidar/config/lidars.yaml`):
  - 192.168.42.0/24, host .102
  - front lidar .200 (6699/7788/6688)
  - rear lidar .201 (6700/7789/6689)
  - cameras .210–.213
- **Bench PC used to build this skill:**
  - host 192.168.77.191/32 on enp2s0
  - lidars .201 and .200
  - Wi-Fi on 192.168.77.0/24, hence the /32

## Checks

```bash
ip -br addr show <iface>; ip route | grep <iface>
ping -c2 -W1 <lidar ip>
curl -s -o /dev/null -w '%{http_code}\n' http://<lidar ip>/     # web page reachable
sudo python3 scripts/discover.py <iface> 6                       # who sends what, where
```
