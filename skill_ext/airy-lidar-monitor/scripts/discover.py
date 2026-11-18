#!/usr/bin/env python3
"""Find RoboSense Airy lidars on a wired interface by listening to the cable.

Needs no IP setup on the PC and no driver: run as root / with CAP_NET_RAW
(sudo, or docker --net=host --cap-add=NET_RAW). Reports, per sending device:
source IP + MAC, destination IP, which UDP ports it sends to and at what rate,
PTP frames, ARP requests it makes (a lidar asking "who has <host ip>" means
the PC lacks that address), status-packet values (rpm, voltage, current,
temperatures), and warns when two lidars send to the same port.

    python3 discover.py <iface> [seconds=6]
"""
import collections, socket, struct, sys, time

iface = sys.argv[1]
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 6
s = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.ntohs(3))
s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
s.bind((iface, 0))
s.settimeout(0.3)
mac = lambda b: ':'.join(f'{x:02x}' for x in b)


def temp(b0, b1):
    v = ((b1 & 0x7F) << 5) + (b0 >> 3)
    return (-v if b1 & 0x80 else v) * 0.0625


dev = collections.defaultdict(lambda: dict(ip=None, ports=collections.Counter(), dst=set(), ptp=0,
                                           arp=set(), difop=None, temp=None, other=collections.Counter()))
end = time.time() + secs
while time.time() < end:
    try:
        f, addr = s.recvfrom(65535)
    except socket.timeout:
        continue
    if addr[2] == socket.PACKET_OUTGOING:
        continue
    src = mac(f[6:12]); et = struct.unpack('!H', f[12:14])[0]; p = f[14:]
    if et == 0x8100:
        et, p = struct.unpack('!H', f[16:18])[0], f[18:]
    d = dev[src]
    if et == 0x0806 and len(p) >= 28:
        d['ip'] = d['ip'] or socket.inet_ntoa(p[14:18])
        if struct.unpack('!H', p[6:8])[0] == 1:
            d['arp'].add(socket.inet_ntoa(p[24:28]))
    elif et == 0x88f7:
        d['ptp'] += 1
    elif et == 0x0800 and len(p) >= 28 and p[9] == 17:
        hl = (p[0] & 15) * 4
        d['ip'] = socket.inet_ntoa(p[12:16]); d['dst'].add(socket.inet_ntoa(p[16:20]))
        dp = struct.unpack('!H', p[hl + 2:hl + 4])[0]
        u = p[hl + 8:]
        d['ports'][dp] += 1
        if len(u) == 1248 and u[:4] == b'\xa5\xff\x00\x5a' and len(u) >= 1092:      # DIFOP
            vm, v12, _, _, im = struct.unpack_from('>5H', u, 1066)
            d['difop'] = dict(port=dp, rpm=struct.unpack_from('>H', u, 8)[0], v=vm / 100, v12=v12 / 100, i=im / 100)
        elif len(u) >= 46 and u[:4] == b'\x55\xaa\x05\x5a':                         # MSOP
            d['temp'] = (temp(u[38], u[39]), temp(u[40], u[41]), dp)
    else:
        d['other'][hex(et)] += 1

port_users = collections.defaultdict(list)
print(f'listened {secs:.0f} s on {iface}\n')
for m, d in sorted(dev.items(), key=lambda kv: -sum(kv[1]['ports'].values())):
    if not d['ports'] and not d['ptp'] and not d['arp']:
        continue
    print(f"{d['ip'] or '?':16s} MAC {m}  -> {', '.join(sorted(d['dst'])) or '-'}")
    for port, n in sorted(d['ports'].items()):
        kind = ''
        if d['temp'] and port == d['temp'][2]: kind = 'points (MSOP)'
        elif d['difop'] and port == d['difop']['port']: kind = 'status (DIFOP)'
        elif 150 < n / secs < 260: kind = 'IMU?'
        print(f'    UDP port {port:5d}  {n / secs:8.1f} pkt/s  {kind}')
        port_users[port].append(d['ip'])
    if d['ptp']:
        print(f"    PTP frames {d['ptp'] / secs:.1f}/s")
    if d['arp']:
        print(f"    asks ARP who-has {', '.join(sorted(d['arp']))}  (if no data flows: the PC lacks that address)")
    if d['difop']:
        x = d['difop']
        print(f"    status: set {x['rpm']} rpm, input {x['v']:.2f} V, 12V rail {x['v12']:.2f} V, {x['i']:.2f} A "
              f"(~{x['v'] * x['i']:.1f} W; 0.01 scale inferred)")
    if d['temp']:
        print(f"    temperature: main board {d['temp'][0]:.2f} C, top board {d['temp'][1]:.2f} C")
    print()
clash = {p: ips for p, ips in port_users.items() if len(set(ips)) > 1}
for p, ips in clash.items():
    print(f'!! port {p} is used by {", ".join(sorted(set(ips)))}: the driver cannot tell them apart. '
          'Give each lidar its own ports in its web page.')
if not dev:
    print('no frames at all: cable, link, or lidar power')
