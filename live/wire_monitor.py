#!/usr/bin/env python3
"""Measure two Airy lidars straight off the cable (AF_PACKET, needs CAP_NET_RAW).

Independent of ROS / the driver. Every second appends one JSON line to <out>:
  per lidar: point/imu/status packet rates, lost packets (from the lidar's own
  sequence counters), measured rotation speed (from the lidar's own packet
  timestamps), set rpm, input/12V voltage, current, power, board temperatures.
  plus: frames this sniffer itself dropped (so they are not blamed on the lidar).

    python3 wire_monitor.py <iface> <out.jsonl>
"""
import json, socket, struct, sys, time

LIDARS = {  # source ip -> (name, msop, difop, imu)
    '192.168.77.201': ('airy1', 6700, 7789, 6689),
    '192.168.77.200': ('airy2', 6699, 7788, 6688),
}
SOL_PACKET, PACKET_STATISTICS = 263, 6
iface, out_path = sys.argv[1], sys.argv[2]

s = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.ntohs(0x0800))
s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
s.bind((iface, 0))
s.settimeout(0.2)


def temp(b0, b1):
    v = ((b1 & 0x7F) << 5) + (b0 >> 3)
    return round((-v if b1 & 0x80 else v) * 0.0625, 4)


def fresh():
    return dict(msop=0, msop_lost=0, msop_bad=0, imu=0, imu_lost=0, imu_bad=0, difop=0,
                periods=[], t_main=None, t_top=None)


port_kind = {}
for ip, (name, m, d, i) in LIDARS.items():
    port_kind[(ip, m)] = (name, 'msop'); port_kind[(ip, d)] = (name, 'difop'); port_kind[(ip, i)] = (name, 'imu')
cur = {n: fresh() for n, *_ in LIDARS.values()}
state = {n: dict(msop_cnt=None, imu_cnt=None, last_az=None, wrap_ts=None, difop=None) for n, *_ in LIDARS.values()}
totals = {n: dict(msop_lost=0, imu_lost=0) for n, *_ in LIDARS.values()}

out = open(out_path, 'a', buffering=1)
next_t = int(time.time()) + 1
while True:
    try:
        f = s.recv(2048)
    except socket.timeout:
        f = None
    if f is not None and len(f) > 42 and f[23] == 17:
        hl = (f[14] & 15) * 4
        src = socket.inet_ntoa(f[26:30])
        dport = (f[14 + hl + 2] << 8) | f[14 + hl + 3]
        k = port_kind.get((src, dport))
        if k:
            name, kind = k
            d = f[14 + hl + 8:]
            c, st = cur[name], state[name]
            if kind == 'msop' and len(d) >= 46:
                c['msop'] += 1
                cnt = struct.unpack_from('>I', d, 12)[0]
                if st['msop_cnt'] is not None:
                    gap = (cnt - st['msop_cnt']) & 0xFFFFFFFF
                    if 1 < gap < 1_000_000:
                        c['msop_lost'] += gap - 1
                    elif gap != 1:
                        c['msop_bad'] += 1          # reorder / counter reset
                st['msop_cnt'] = cnt
                c['t_main'] = (d[38], d[39]); c['t_top'] = (d[40], d[41])
                az = struct.unpack_from('>H', d, 44)[0]
                if st['last_az'] is not None and az + 18000 < st['last_az']:   # new rotation
                    sec = int.from_bytes(d[20:26], 'big'); usec = int.from_bytes(d[26:30], 'big')
                    ts = sec + usec * 1e-6
                    if st['wrap_ts'] is not None and 0 < ts - st['wrap_ts'] < 1:
                        c['periods'].append(ts - st['wrap_ts'])
                    st['wrap_ts'] = ts
                st['last_az'] = az
            elif kind == 'imu' and len(d) >= 8:
                c['imu'] += 1
                cnt = struct.unpack_from('>I', d, len(d) - 6)[0]
                if st['imu_cnt'] is not None:
                    gap = (cnt - st['imu_cnt']) & 0xFFFFFFFF
                    if 1 < gap < 1_000_000:
                        c['imu_lost'] += gap - 1
                    elif gap != 1:
                        c['imu_bad'] += 1
                st['imu_cnt'] = cnt
            elif kind == 'difop' and len(d) >= 1092:
                c['difop'] += 1
                v_main, v_12, v_mcu, v_1, i_main = struct.unpack_from('>5H', d, 1066)
                st['difop'] = dict(rpm_set=struct.unpack_from('>H', d, 8)[0],
                                   v_main=v_main / 100, v_12v=v_12 / 100, i_main=i_main / 100,
                                   power=round(v_main / 100 * i_main / 100, 3), t_difop=time.time())
    now = time.time()
    if now >= next_t:
        try:
            pk, dr = struct.unpack('II', s.getsockopt(SOL_PACKET, PACKET_STATISTICS, 8))
        except OSError:
            dr = None
        row = {'t': next_t, 'sniff_drops': dr, 'lidars': {}}
        for name, c in cur.items():
            st = state[name]
            totals[name]['msop_lost'] += c['msop_lost']; totals[name]['imu_lost'] += c['imu_lost']
            p = c['periods']
            r = dict(msop=c['msop'], msop_lost=c['msop_lost'], msop_bad=c['msop_bad'],
                     msop_lost_total=totals[name]['msop_lost'],
                     imu=c['imu'], imu_lost=c['imu_lost'], imu_bad=c['imu_bad'],
                     imu_lost_total=totals[name]['imu_lost'], difop=c['difop'],
                     rot_hz=round(len(p) / sum(p), 4) if p else None,
                     rpm_meas=round(60 * len(p) / sum(p), 2) if p else None,
                     period_jitter_ms=round((max(p) - min(p)) * 1e3, 3) if len(p) > 1 else None,
                     t_main=temp(*c['t_main']) if c['t_main'] else None,
                     t_top=temp(*c['t_top']) if c['t_top'] else None)
            dif = st['difop']
            if dif and now - dif['t_difop'] < 3:
                r.update({k: v for k, v in dif.items() if k != 't_difop'})
            row['lidars'][name] = r
        out.write(json.dumps(row) + '\n')
        cur = {n: fresh() for n in cur}
        next_t += 1
        if now - next_t > 5:                # fell behind (suspend etc): resync
            next_t = int(now) + 1
