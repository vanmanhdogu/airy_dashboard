#!/usr/bin/env python3
"""Standby / run control for RoboSense Airy lidars through their built-in web page.

The Airy's web page (http://<lidar ip>/Parameter_Setting.html) has "Operation Mode:
Standby / High-Performance" (field OpM, 0 = standby, 1 = run). Standby stops the motor and
the laser: no point or IMU packets, status packets (DIFOP) and the web page keep going.

The page has no separate API: it loads every setting from /setting_data.json and saves by
POSTing the whole form back. So to change OpM we read the current settings, rebuild exactly
the form the browser would send (same fields, same order), change only OpM and POST it.
Anything unexpected in the settings (unknown value for a list field) aborts before the POST.

    python3 airy_web.py <ip>                    # show the current mode
    python3 airy_web.py <ip> standby|run        # switch
    python3 airy_web.py <ip> standby --dry-run  # print the form it would send
"""
import json, threading, time, urllib.parse, urllib.request

TIMEOUT = 3
MODES = {'standby': '0', 'run': '1'}
MODE_NAME = {v: k for k, v in MODES.items()}

# The form on Parameter_Setting.html, in document order (fields inside HTML comments left out,
# like the browser does). ('select', name, allowed values) | ('text', name) | ('raw', name, value)
_FORM = [
    ('raw', 'tabs-two', 'on'),
    ('text', 'SrcIp'), ('text', 'SrcMask'), ('text', 'SrcGateWay'), ('text', 'DstIp'),
    ('text', 'MPort'), ('text', 'MPort1'), ('text', 'DPort'),
    ('select', 'ReMode', '0123'),
    ('text', 'FovS'), ('text', 'FovE'),
    ('select', 'SynSrc', '01234'),
    ('text', 'PDomNum'),
    ('select', 'RTPdelay', '01'), ('select', 'NoLeap', '01'),
    ('text', 'SyTOut'), ('text', 'UToL'), ('text', 'LToU'),
    ('select', 'OpM', '01'),
    ('text', 'PhaLockSet'),
    ('select', 'GPSBRate', ('3', '4', '5', '6', '7', '8', '9', '10')),
    ('select', 'MotorR', '01'),
    ('select', 'ImuCtrl', '01'),
    ('text', 'IPort'),
    ('select', 'ImuOutR', '01234'),
    ('select', 'AccRg', '0123'), ('select', 'GyroRg', '0123'),
    ('text', 'Lpf'),
    ('raw', 'ResDefault', '0'),          # "Restore Default": never anything but 0
    ('raw', 'save_param', 'Save'),       # the Save button of the first tab
    ('select', 'RefEn', '0123'),
    ('select', 'TFilLvl', '1234567'),
    ('select', 'BlkRnDis', '012'), ('select', 'Rn', '012'), ('select', 'Blk', '012'),
    ('text', 'FSAgl'),
    ('select', 'PoorMask', '01'), ('select', 'DZ10cm', '01'), ('select', 'C81858993', '01'),
    ('select', 'GapFill', '01'), ('select', 'RanM', '012'), ('select', 'RnFgSwt', '01'),
    ('select', 'NrFuSwt', '01'), ('select', 'L48', '01'),
]


def get_settings(ip, timeout=TIMEOUT):
    with urllib.request.urlopen(f'http://{ip}/setting_data.json', timeout=timeout) as r:
        return json.loads(r.read().decode('latin1'))


def get_mode(ip, timeout=TIMEOUT):
    """'standby', 'run' or raises."""
    opm = str(get_settings(ip, timeout).get('OpM'))
    if opm not in MODE_NAME:
        raise ValueError(f'unknown operation mode {opm!r}')
    return MODE_NAME[opm]


def build_form(settings, mode):
    """The form fields (list of (name, value)) the web page would send with OpM = mode."""
    s = dict(settings)
    s['OpM'] = MODES[mode]
    out = []
    for f in _FORM:
        kind, name = f[0], f[1]
        if kind == 'raw':
            out.append((name, f[2]))
            continue
        if name not in s:
            raise ValueError(f'lidar settings have no {name}: different firmware, not touching it')
        v = str(s[name])
        if kind == 'select' and v not in f[2]:
            raise ValueError(f'unexpected {name}={v!r}: different firmware, not touching it')
        out.append((name, v))
    # third tab (pulse output): checkboxes are sent only when ticked, the angle fields only when enabled
    out.append(('mode2', 'Mode2') if str(s.get('TriggerMode')) == '0' else ('mode1', 'Mode1(+25%)'))
    if str(s.get('GroupSelect')) != '0':
        out += [('first_group_switch', 'ON'), ('first_start_angle', str(s['PulseStart'])),
                ('first_pulse_width', str(s['PulseWidth'])), ('first_pulse_step', str(s['PulseStep']))]
    return out


def set_mode(ip, mode, wait=45, dry_run=False):
    """Switch the lidar at ip to 'standby' or 'run' and wait until it reports that mode."""
    if mode not in MODES:
        raise ValueError(mode)
    before = get_settings(ip)
    form = build_form(before, mode)
    if dry_run:
        return form
    body = urllib.parse.urlencode(form).encode()
    req = urllib.request.Request(f'http://{ip}/Parameter_Setting.html', data=body, method='POST',
                                 headers={'Content-Type': 'application/x-www-form-urlencoded'})
    try:
        with urllib.request.urlopen(req, timeout=10) as r:
            r.read()
    except OSError:
        pass                                   # some firmware drops the connection after saving
    end = time.time() + wait
    while time.time() < end:                   # the lidar may restart after a save: retry
        time.sleep(1)
        try:
            after = get_settings(ip)
        except (OSError, ValueError):
            continue
        if str(after.get('OpM')) == MODES[mode]:
            changed = [k for k in before if k != 'OpM' and str(after.get(k)) != str(before[k])]
            if changed:
                raise RuntimeError(f'mode set, but these settings changed too: {", ".join(changed)}')
            return form
    raise TimeoutError(f'lidar {ip} did not report {mode} within {wait} s')


class LidarControl:
    """Mode of every lidar (polled), plus switch requests running in the background."""

    def __init__(self, lidars, on_change=None, poll_s=5, dry_run=False):
        self.ips = {l['name']: l.get('ip', '') for l in lidars}
        self.on_change = on_change              # on_change(name, mode, text, level)
        self.poll_s, self.dry_run = poll_s, dry_run
        self.lock = threading.Lock()
        # since: when the current mode was first seen; resumed: when it last went standby -> run
        self.st = {n: dict(mode=None, busy=None, error=None, since=None, resumed=None) for n in self.ips}

    def start(self):
        threading.Thread(target=self._poll, daemon=True).start()
        return self

    def state(self):
        with self.lock:
            return {n: dict(v) for n, v in self.st.items()}

    def mode(self, name):
        with self.lock:
            return self.st[name]['mode']

    def _set(self, name, mode):
        with self.lock:
            s = self.st[name]
            old = s['mode']
            s['mode'] = mode
            if mode != old:
                s['since'] = time.time()
                if old == 'standby' and mode == 'run':
                    s['resumed'] = s['since']
        if old is not None and mode is not None and mode != old and self.on_change:
            self.on_change(name, mode, None, None)

    def _poll(self):
        while True:
            for n, ip in self.ips.items():
                with self.lock:
                    if self.st[n]['busy']:
                        continue
                if not ip:
                    continue
                try:
                    m = get_mode(ip)
                except (OSError, ValueError):
                    m = None                     # web page not reachable: mode unknown
                if not self.dry_run:
                    self._set(n, m)
                elif self.mode(n) is None:
                    self._set(n, m or 'run')
            time.sleep(self.poll_s)

    def request(self, names, mode):
        """Start switching names to mode. Returns the names actually started."""
        started = []
        for n in names:
            with self.lock:
                if n not in self.st or not self.ips.get(n) or self.st[n]['busy']:
                    continue
                self.st[n].update(busy=mode, error=None)
            threading.Thread(target=self._switch, args=(n, mode), daemon=True).start()
            started.append(n)
        return started

    def _switch(self, n, mode):
        err = None
        try:
            if self.dry_run:
                set_mode(self.ips[n], mode, dry_run=True)
                time.sleep(2)
            else:
                set_mode(self.ips[n], mode)
        except Exception as e:                    # report any failure on the dashboard
            err = str(e) or e.__class__.__name__
        with self.lock:
            self.st[n].update(busy=None, error=err)
        if err:
            if self.on_change:
                self.on_change(n, None, f'switching to {mode} failed: {err}', 'bad')
        else:
            self._set(n, mode)


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('ip')
    ap.add_argument('mode', nargs='?', choices=list(MODES))
    ap.add_argument('--dry-run', action='store_true')
    a = ap.parse_args()
    if not a.mode:
        print(get_mode(a.ip))
    elif a.dry_run:
        print(urllib.parse.urlencode(set_mode(a.ip, a.mode, dry_run=True)).replace('&', '\n'))
    else:
        set_mode(a.ip, a.mode)
        print(f'{a.ip}: {get_mode(a.ip)}')
