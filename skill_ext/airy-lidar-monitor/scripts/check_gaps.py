#!/usr/bin/env python3
"""Cross-check: were scans missing in ROS (driver), or only in the live monitor?

Reads point-cloud header stamps from one or more .mcap files (recorder = independent
subscriber) and compares gaps > 150 ms with the gaps the live monitor reported.
Topics ending in /points are grouped by their first path element (/front_lidar/points -> front_lidar).
A closed split file (<bag>_0.mcap) can be checked while recording continues.

    python3 check_gaps.py <run_dir> <file.mcap> [...]
"""
import json, struct, sys
import rosbag2_py

run, files = sys.argv[1], sys.argv[2:]
import collections
stamps = collections.defaultdict(list)
for f in files:
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=f, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    while r.has_next():
        topic, data, t = r.read_next()
        if topic.endswith('/points'):
            sec, nsec = struct.unpack_from('<iI', data, 4)
            stamps[topic.split('/')[1]].append(sec + nsec * 1e-9)
rows = [json.loads(l) for l in open(f'{run}/live.jsonl')]
for n, s in sorted(stamps.items()):
    s.sort()
    t0, t1 = s[0], s[-1]
    bag_gaps = [(round(b, 1), round((b - a) * 1e3)) for a, b in zip(s, s[1:]) if b - a > 0.15]
    expected = round((t1 - t0) * 10) + 1
    mon = [r for r in rows if t0 + 1 <= r['t'] <= t1]
    mon_gaps = [(r['t'], r[n]['pts_gap_ms']) for r in mon if n in r and (r[n]['pts_gap_ms'] or 0) > 150]
    print(f'{n}: bag has {len(s)} clouds over {t1 - t0:.0f} s (expected {expected}), '
          f'gaps >150 ms in bag: {len(bag_gaps)}, in live monitor: {len(mon_gaps)}')
    for b, g in bag_gaps[:15]:
        print(f'   bag gap {g} ms ending {b:.1f}')
    for t, g in mon_gaps[:15]:
        print(f'   monitor gap {g} ms in second {t}')
