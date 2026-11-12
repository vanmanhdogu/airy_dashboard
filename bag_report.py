#!/usr/bin/env python3
"""Report per-topic message count / frequency and Airy temperatures from a bag.

    python3 bag_report.py <bag_dir>
"""
import sys
import statistics

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Temperature

TEMP_TOPICS = ('/airy/temperature', '/airy/temperature_topboard')

bag = sys.argv[1]
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=bag), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in reader.get_all_topics_and_types()}

stamps = {name: [] for name in types}
temps = {name: [] for name in TEMP_TOPICS}
while reader.has_next():
    topic, data, t = reader.read_next()
    stamps[topic].append(t * 1e-9)
    if topic in temps:
        temps[topic].append(deserialize_message(data, Temperature).temperature)

print(f'bag: {bag}\n')
print(f'{"topic":32s} {"type":32s} {"count":>7s} {"Hz":>8s} {"max gap ms":>11s}')
for name in sorted(types):
    s = stamps[name]
    if len(s) > 1:
        hz = (len(s) - 1) / (s[-1] - s[0])
        gap = max(b - a for a, b in zip(s, s[1:])) * 1e3
        print(f'{name:32s} {types[name]:32s} {len(s):7d} {hz:8.2f} {gap:11.1f}')
    else:
        print(f'{name:32s} {types[name]:32s} {len(s):7d} {"-":>8s} {"-":>11s}')

print()
for name, v in temps.items():
    if v:
        print(f'{name:32s} min {min(v):6.2f}  mean {statistics.mean(v):6.2f}  '
              f'max {max(v):6.2f} degC  (first {v[0]:.2f}, last {v[-1]:.2f})')
    else:
        print(f'{name:32s} no data')
