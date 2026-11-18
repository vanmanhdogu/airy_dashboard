#!/usr/bin/env python3
"""Live monitor for RoboSense Airy lidars: ROS side + PC health + wire data, served as a web dashboard.

Every second it builds one row per lidar from
  - ROS:  points topic (rate, longest gap, valid points, scan width, delay), imu topic (rate, gap)
  - wire: <run>/wire.jsonl written by wire_monitor.py (motor speed, lost packets, power, temperatures)
  - PC:   CPU, UDP socket drops, network-card drops, network Mbit/s, driver CPU, disk
appends it to <run>/live.csv + live.jsonl, and serves http://127.0.0.1:<port>/ (dashboard.html)
with /data?from=N returning rows N.. as JSON, and POST /control {"lidars": [names], "mode": "standby"|"run"}
switching lidars between standby and running through their web page (airy_web.py).

    python3 live_monitor.py --lidars lidars.json --run <dir> --duration <secs|0> [--bag <dir>] [--port 8090]

lidars.json: [{"name", "ip", "points_topic", "imu_topic"}, ...] (written by run_live.sh)
"""
import argparse, collections, csv, json, os, shutil, struct, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2

from airy_web import LidarControl

FULL_COLS = 900                    # Airy: 900 columns x 96 rings per turn at 10 Hz
HERE = os.path.dirname(os.path.abspath(__file__))

ap = argparse.ArgumentParser()
ap.add_argument('--lidars', required=True, help='lidars.json')
ap.add_argument('--run', required=True)
ap.add_argument('--duration', type=int, required=True)
ap.add_argument('--bag', default='')
ap.add_argument('--port', type=int, default=8090)
ap.add_argument('--iface', default='enp2s0')
ap.add_argument('--continue', dest='cont', default='', help='previous run dir: keep its timeline')
ap.add_argument('--control', choices=('on', 'off', 'dry-run'), default='on',
                help='standby/run buttons on the dashboard (dry-run: buttons work, nothing is sent to the lidars)')
args, ros_args = ap.parse_known_args()
LIDARS = json.load(open(args.lidars))
NAMES = [l['name'] for l in LIDARS]

lock = threading.Lock()
ROWS, EVENTS = [], []
START = time.time()
if args.cont:                     # continue the timeline of a previous run (duration 0 = until stopped)
    START = json.load(open(os.path.join(args.cont, 'meta.json')))['start']
    for fn, dst in (('live.jsonl', ROWS), ('events.jsonl', EVENTS)):
        try:
            dst.extend(json.loads(l) for l in open(os.path.join(args.cont, fn)) if l.strip())
        except OSError:
            pass
META = dict(start=START, duration=args.duration, run=args.run, bag=args.bag,
            lidars=[dict(name=l['name'], ip=l.get('ip', '')) for l in LIDARS],
            finished=False, continues=args.cont or None, control=args.control != 'off')
SPINUP_S = 30                      # after leaving standby: motor spin-up, no alarms yet


# ---------------------------------------------------------------- ROS side
class Stats:
    def __init__(self):
        self.pts = collections.deque()      # (recv_time, valid, width, delay_ms)
        self.imu = collections.deque()      # recv_time
        self.last_pts = None
        self.last_imu = None
        self.pts_total = 0
        self.imu_total = 0


def _al(pos, n):
    """CDR alignment (relative to the byte after the 4-byte encapsulation header)."""
    r = (pos - 4) % n
    return pos + (n - r if r else 0)


def parse_cloud(b):
    """Read stamp, size and x values from a serialized PointCloud2 without deserializing it."""
    sec, nsec = struct.unpack_from('<iI', b, 4)
    pos = 12
    ln = struct.unpack_from('<I', b, pos)[0]; pos += 4 + ln
    pos = _al(pos, 4)
    height, width, nf = struct.unpack_from('<III', b, pos); pos += 12
    xo = None
    for _ in range(nf):
        ln = struct.unpack_from('<I', b, pos)[0]; name = b[pos + 4:pos + 3 + ln]; pos += 4 + ln
        pos = _al(pos, 4); off = struct.unpack_from('<I', b, pos)[0]; pos += 4
        pos += 1                                   # datatype
        pos = _al(pos, 4); pos += 4                # count
        if name == b'x':
            xo = off
    pos += 1                                       # is_bigendian
    pos = _al(pos, 4)
    point_step, row_step, dlen = struct.unpack_from('<III', b, pos); pos += 12
    return sec + nsec * 1e-9, height, width, xo, point_step, pos, dlen


class RosSide(Node):
    # Raw (serialized) subscriptions: deserializing two 2.6 MB clouds per 100 ms in Python
    # made this monitor miss messages. Only the bytes needed are read.
    def __init__(self):
        super().__init__('airy_live_monitor')
        self.s = {n: Stats() for n in NAMES}
        # RELIABLE like the recorder: best-effort loses some of the 2.6 MB clouds (split into many fragments)
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        for l in LIDARS:
            n = l['name']
            self.create_subscription(PointCloud2, l['points_topic'], lambda m, n=n: self.on_pts(n, m), qos, raw=True)
            if l.get('imu_topic'):
                self.create_subscription(Imu, l['imu_topic'], lambda m, n=n: self.on_imu(n, m), qos, raw=True)

    def on_pts(self, n, b):
        now = time.time()
        try:
            stamp, h, w, xo, step, pos, dlen = parse_cloud(b)
        except struct.error:
            return
        if stamp < 1e9:
            return
        npts = h * w
        valid = None
        if xo is not None and step and dlen >= npts * step:
            a = np.frombuffer(b, dtype=np.uint8, count=npts * step, offset=pos).reshape(npts, step)
            x = a[:, xo:xo + 4].copy().view(np.float32).ravel()
            valid = float(np.count_nonzero(np.isfinite(x) & (x != 0))) / max(npts, 1) * 100
        cols = npts // 96               # 96 rings; a full turn is 900 columns
        with lock:
            s = self.s[n]
            s.pts.append((now, valid, cols, (now - stamp) * 1e3, stamp))
            s.pts_total += 1

    def on_imu(self, n, b):
        sec, nsec = struct.unpack_from('<iI', b, 4)
        if sec < 1e9:
            return
        with lock:
            s = self.s[n]
            s.imu.append((time.time(), sec + nsec * 1e-9))
            s.imu_total += 1


# ---------------------------------------------------------------- PC side
def read_proc():
    cpu = [int(x) for x in open('/proc/stat').readline().split()[1:]]
    udp = {}
    lines = [l.split() for l in open('/proc/net/snmp') if l.startswith('Udp:')]
    if len(lines) >= 2:
        udp = dict(zip(lines[0][1:], map(int, lines[1][1:])))
    nic = None
    for l in open('/proc/net/dev'):
        if l.strip().startswith(args.iface + ':'):
            v = [int(x) for x in l.split(':', 1)[1].split()]
            nic = dict(rx_bytes=v[0], rx_errs=v[2], rx_drop=v[3], rx_fifo=v[4])
    return cpu, udp, nic


def driver_pids():
    pids = []
    for p in os.listdir('/proc'):
        if p.isdigit():
            try:
                if open(f'/proc/{p}/comm').read().strip() == 'rslidar_sdk_nod':
                    pids.append(int(p))
            except OSError:
                pass
    return pids


def proc_ticks(pid):
    try:
        f = open(f'/proc/{pid}/stat').read().rsplit(')', 1)[1].split()
        return int(f[11]) + int(f[12]), int(f[21])  # utime+stime, rss pages
    except (OSError, IndexError, TypeError):
        return None, None


def dir_size(path):
    tot = 0
    try:
        for e in os.scandir(path):
            if e.is_file():
                tot += e.stat().st_size
    except OSError:
        pass
    return tot


# ---------------------------------------------------------------- wire side
class WireTail:
    def __init__(self, path):
        self.path, self.pos, self.latest = path, 0, None

    def poll(self):
        try:
            with open(self.path) as f:
                f.seek(self.pos)
                for line in f:
                    if line.endswith('\n'):
                        try:
                            self.latest = json.loads(line)
                        except ValueError:
                            pass
                        self.pos += len(line.encode())
        except OSError:
            pass
        return self.latest


# ---------------------------------------------------------------- sampler
def add_event(t, lidar, kind, text, level):
    e = dict(t=t, el=round(t - START), lidar=lidar, kind=kind, text=text, level=level)
    with lock:
        EVENTS.append(e)
    with open(os.path.join(args.run, 'events.jsonl'), 'a') as f:
        f.write(json.dumps(e) + '\n')


def on_mode_change(name, mode, text, level):
    if text is None:
        text, level = (('put in standby: motor and laser off', 'warn') if mode == 'standby'
                       else ('running again (left standby)', 'ok'))
    add_event(time.time(), name, 'mode', text, level)


CTL = LidarControl(LIDARS, on_change=on_mode_change, dry_run=args.control == 'dry-run')


def sampler(node):
    wire = WireTail(os.path.join(args.run, 'wire.jsonl'))
    hz_ticks = os.sysconf('SC_CLK_TCK')
    page = os.sysconf('SC_PAGE_SIZE')
    prev_cpu, prev_udp, prev_nic = read_proc()
    prev_dt = {}
    csv_f = open(os.path.join(args.run, 'live.csv'), 'w', newline='')
    jsonl = open(os.path.join(args.run, 'live.jsonl'), 'w', buffering=1)
    writer = None
    active = {}                          # (lidar, kind) -> True while an event is ongoing
    base_v = {}
    nxt = int(time.time()) + 1
    while rclpy.ok():
        time.sleep(max(0, nxt - time.time()))
        t = nxt
        nxt += 1
        el = t - START
        cpu, udp, nic = read_proc()
        dc = [a - b for a, b in zip(cpu, prev_cpu)]
        busy = 100 * (1 - (dc[3] + dc[4]) / max(sum(dc), 1))
        pc = dict(cpu=round(busy, 1),
                  udp_rcvbuf_err=udp.get('RcvbufErrors', 0) - prev_udp.get('RcvbufErrors', 0),
                  udp_in_err=udp.get('InErrors', 0) - prev_udp.get('InErrors', 0))
        if nic and prev_nic:
            # rx_drop is NOT used: Linux counts frames of unhandled protocols there (the lidars' PTP
            # messages, 2/s). rx_fifo = receive buffer overruns = real hardware drops.
            pc.update(nic_drop=nic['rx_fifo'] - prev_nic['rx_fifo'],
                      nic_err=nic['rx_errs'] - prev_nic['rx_errs'],
                      mbps=round((nic['rx_bytes'] - prev_nic['rx_bytes']) * 8 / 1e6, 2))
        prev_cpu, prev_udp, prev_nic = cpu, udp, nic
        # all rslidar_sdk_node processes together (100 % = one core)
        cur_dt, rss_tot = {}, 0
        for pid in driver_pids():
            tk, rss = proc_ticks(pid)
            if tk is not None:
                cur_dt[pid] = tk; rss_tot += rss
        if cur_dt:
            common = [p for p in cur_dt if p in prev_dt]
            pc['driver_cpu'] = round(sum(cur_dt[p] - prev_dt[p] for p in common) / hz_ticks * 100, 1) if common else None
            pc['driver_mem_mb'] = round(rss_tot * page / 2**20, 1)
            pc['drivers'] = len(cur_dt)
        prev_dt = cur_dt
        du = shutil.disk_usage(args.run)
        pc['disk_free_gb'] = round(du.free / 2**30, 1)
        if args.bag:
            pc['bag_gb'] = round(dir_size(args.bag) / 2**30, 2)
        w = wire.poll()
        pc['sniff_drops'] = w.get('sniff_drops') if w else None
        wire_age = time.time() - w['t'] if w else None

        row = dict(t=t, el=round(el), pc=pc)
        with lock:
            for n in NAMES:
                s = node.s[n]
                while s.pts and s.pts[0][0] < t - 10:
                    s.pts.popleft()
                while s.imu and s.imu[0][0] < t - 10:
                    s.imu.popleft()
                p1 = [p for p in s.pts if p[0] >= t - 1]
                p5 = [p for p in s.pts if p[0] >= t - 5]
                i1 = [x for x in s.imu if x[0] >= t - 1]
                i5 = [x for x in s.imu if x[0] >= t - 5]
                # gaps from the driver's message stamps (host time when the driver built the
                # message), so delays inside this monitor do not show up as gaps
                ts = [p[4] for p in p1]
                prev = [p[4] for p in s.pts if p[0] < t - 1]
                seq = ([prev[-1]] if prev else []) + ts
                gap = max((b - a for a, b in zip(seq, seq[1:])), default=None)
                if not p1 and s.pts:
                    gap = t - s.pts[-1][0]
                elif not s.pts:
                    gap = None
                iseq = [x[1] for x in s.imu if x[0] < t - 1][-1:] + [x[1] for x in i1]
                igap = max((b - a for a, b in zip(iseq, iseq[1:])), default=None)
                vals = [p[1] for p in p1 if p[1] is not None]
                r = dict(
                    pts_hz=round(len(p5) / 5, 2) if el >= 5 else len(p1),
                    pts_n=len(p1),
                    pts_gap_ms=round(gap * 1e3, 1) if gap is not None else None,
                    short_scans=sum(1 for p in p1 if p[2] < FULL_COLS),
                    min_cols=min((p[2] for p in p1), default=None),
                    valid_pct=round(sum(vals) / len(vals), 2) if vals else None,
                    delay_ms=round(sum(p[3] for p in p1) / len(p1), 1) if p1 else None,
                    imu_hz=round(len(i5) / 5, 1) if el >= 5 else len(i1),
                    imu_gap_ms=round(igap * 1e3, 1) if igap is not None else None,
                )
                lw = (w or {}).get('lidars', {}).get(n) if wire_age is not None and wire_age < 4 else None
                if lw:
                    r.update({k: lw.get(k) for k in (
                        'msop', 'msop_lost', 'msop_bad', 'msop_lost_total', 'imu', 'imu_lost', 'imu_lost_total',
                        'rpm_meas', 'period_jitter_ms', 'rpm_set', 'v_main', 'v_12v', 'i_main',
                        'power', 't_main', 't_top')})
                row[n] = r
        ctl = CTL.state() if args.control != 'off' else {}
        for n in NAMES:
            c = ctl.get(n, {})
            row[n]['mode'] = c.get('mode')
        # ---- events (start / end of each problem)
        for n in NAMES:
            r = row[n]
            c = ctl.get(n, {})
            # in standby on purpose, switching, or still spinning up: the lidar sends nothing, so
            # every lidar check would fire. Close the open ones and skip them until it runs again.
            quiet = bool(c.get('busy') or c.get('mode') == 'standby'
                         or (c.get('resumed') and c.get('mode') == 'run' and t - c['resumed'] < SPINUP_S))
            if quiet:
                for (ln, kind) in list(active):
                    if ln == n and active[(ln, kind)]:
                        active[(ln, kind)] = False
                continue
            if r.get('v_main') and n not in base_v and el > 20:
                base_v[n] = r['v_main']
            checks = {
                'no_points': (r['pts_n'] == 0 and el > 10, 'no point clouds in ROS', 'bad'),
                'hz': (el > 10 and r['pts_n'] > 0 and abs(r['pts_hz'] - 10) > 0.3,
                       f"point cloud rate {r['pts_hz']} Hz", 'warn'),
                'gap': (el > 10 and r['pts_gap_ms'] is not None and r['pts_gap_ms'] > 150,
                        f"point cloud gap {r['pts_gap_ms']} ms", 'warn'),
                'imu_gap': (r['imu_gap_ms'] is not None and r['imu_gap_ms'] > 20 and el > 10,
                            f"IMU gap {r['imu_gap_ms']} ms", 'warn'),
                'lost': ((r.get('msop_lost') or 0) > 0, f"{r.get('msop_lost')} point packets lost on the cable", 'warn'),
                'imu_lost': ((r.get('imu_lost') or 0) > 0, f"{r.get('imu_lost')} IMU packets lost on the cable", 'warn'),
                'restart': (el > 10 and (r.get('msop_bad') or 0) > 0,
                            'lidar restarted: its packet counter started over (power cut/dip or reset)', 'bad'),
                'no_wire': (w is not None and r.get('msop') in (None, 0) and el > 10, 'no packets from the lidar on the cable', 'bad'),
                'rpm': (r.get('rpm_meas') is not None and abs(r['rpm_meas'] - 600) > 6,
                        f"motor speed {r.get('rpm_meas')} rpm", 'bad'),
                'volt': (n in base_v and r.get('v_main') is not None and base_v[n] - r['v_main'] > 0.3,
                         f"input voltage dropped to {r.get('v_main')} V (start {base_v.get(n)} V)", 'warn'),
                'temp': ((r.get('t_main') or 0) >= 90, f"main board {r.get('t_main')} °C", 'warn'),
            }
            for kind, (on, text, level) in checks.items():
                if on and not active.get((n, kind)):
                    add_event(t, n, kind, text, level)
                    active[(n, kind)] = True
                elif not on and active.get((n, kind)):
                    add_event(t, n, kind + '_ok', kind.replace('_', ' ') + ' back to normal', 'ok')
                    active[(n, kind)] = False
        for kind, val, text in (('udp', pc['udp_rcvbuf_err'], 'UDP packets dropped by the PC (socket buffer full)'),
                                ('nic', pc.get('nic_drop', 0), 'packets dropped by the network card'),
                                ('sniff', pc.get('sniff_drops') or 0, 'wire monitor itself dropped frames')):
            on = bool(val and val > 0)
            if on and not active.get(('pc', kind)):
                add_event(t, 'pc', kind, f'{val}/s {text}', 'warn')
            elif not on and active.get(('pc', kind)):
                add_event(t, 'pc', kind + '_ok', text.split('(')[0].strip() + ': stopped', 'ok')
            active[('pc', kind)] = on

        flat = dict(t=t, elapsed_s=row['el'])
        for n in NAMES:
            flat.update({f'{n}_{k}': v for k, v in row[n].items()})
        flat.update({f'pc_{k}': v for k, v in pc.items()})
        if writer is None:
            writer = csv.DictWriter(csv_f, fieldnames=list(flat), extrasaction='ignore')
            writer.writeheader()
        writer.writerow(flat)
        csv_f.flush()
        jsonl.write(json.dumps(row) + '\n')
        with lock:
            ROWS.append(row)
            if args.duration and el >= args.duration:
                META['finished'] = True


# ---------------------------------------------------------------- web
class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype):
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        u = urlparse(self.path)
        if u.path in ('/', '/index.html'):
            with open(os.path.join(HERE, 'dashboard.html'), 'rb') as f:
                self._send(200, f.read(), 'text/html; charset=utf-8')
        elif u.path == '/data':
            q = parse_qs(u.query)
            frm = int(q.get('from', ['0'])[0])
            efrm = int(q.get('efrom', ['0'])[0])
            with lock:
                body = json.dumps(dict(meta=META, now=time.time(), total=len(ROWS), rows=ROWS[frm:],
                                       etotal=len(EVENTS), events=EVENTS[efrm:],
                                       ctl=CTL.state() if args.control != 'off' else None))
            self._send(200, body.encode(), 'application/json')
        else:
            self._send(404, b'not found', 'text/plain')

    def do_POST(self):
        # JSON only: a cross-site page cannot send that without a CORS preflight this server never allows
        if urlparse(self.path).path != '/control' or args.control == 'off':
            return self._send(404, b'not found', 'text/plain')
        if self.headers.get('Content-Type', '').split(';')[0].strip() != 'application/json':
            return self._send(415, b'json only', 'text/plain')
        try:
            req = json.loads(self.rfile.read(int(self.headers.get('Content-Length', 0))) or b'{}')
            mode, names = req['mode'], req.get('lidars') or NAMES
            assert mode in ('standby', 'run') and all(n in NAMES for n in names)
        except (ValueError, KeyError, AssertionError, TypeError):
            return self._send(400, b'bad request', 'text/plain')
        started = CTL.request(names, mode)
        if started:
            add_event(time.time(), ', '.join(started), 'mode_req',
                      f"{'standby' if mode == 'standby' else 'run'} requested from the dashboard", 'ok')
        self._send(202, json.dumps(dict(started=started)).encode(), 'application/json')


def main():
    rclpy.init(args=ros_args)
    node = RosSide()
    srv = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    if args.control != 'off':
        CTL.start()
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    threading.Thread(target=sampler, args=(node,), daemon=True).start()
    with open(os.path.join(args.run, 'meta.json'), 'w') as f:
        json.dump(META, f)
    print(f'dashboard: http://127.0.0.1:{args.port}/', flush=True)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        with open(os.path.join(args.run, 'events.json'), 'w') as f:
            json.dump(EVENTS, f, indent=1)


if __name__ == '__main__':
    main()
