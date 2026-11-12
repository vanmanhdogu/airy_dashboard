#!/usr/bin/env python3
"""Analyse an Airy test bag (one per run_and_record.sh) -> JSON for the report page.

    python3 analyze_bag.py <bag_dir> <out.json> [--workers N]

Reads every .mcap file of the bag in parallel. Per point cloud: receive time, header
stamp, valid points, range/intensity statistics, per-ring validity, and per-pixel
range sums (every 10th frame) for a range-noise estimate. Per IMU message: accel and
gyro. Temperatures from /airy/temperature*. Aggregates to 10 s bins.
"""
import argparse
import collections
import glob
import json
import os
from multiprocessing import Pool

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, PointCloud2, Temperature

PTS, IMU_T = '/rslidar_points', '/rslidar_imu_data'
T_MAIN, T_TOP = '/airy/temperature', '/airy/temperature_topboard'
FULL_COLS = 900         # columns of a full Airy revolution at 10 Hz
NOISE_EVERY = 10          # use every 10th cloud for the per-pixel noise estimate


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def read_file(path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id='mcap'),
           rosbag2_py.ConverterOptions('', ''))
    pc, imu, tm, tt = [], [], [], []
    shape = None
    ring_valid = ring_total = None
    psum = psq = pcnt = None
    sample = None
    heights = {}
    k = 0
    while r.has_next():
        topic, data, t = r.read_next()
        t *= 1e-9
        if topic == PTS:
            m = deserialize_message(data, PointCloud2)
            a = np.frombuffer(m.data, np.float32).reshape(m.height, m.width,
                                                          m.point_step // 4)[:, :, :4]
            x, y, z, it = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
            rng = np.sqrt(x * x + y * y + z * z)
            ok = np.isfinite(rng) & (rng > 0)
            if shape is None:
                shape = (FULL_COLS, m.width)
                ring_valid = np.zeros(m.width); ring_total = 0
                psum = np.zeros(shape); psq = np.zeros(shape); pcnt = np.zeros(shape)
            ring_valid += ok.sum(axis=0); ring_total += m.height
            rv = rng[ok]; iv = it[ok]
            heights[m.height] = heights.get(m.height, 0) + 1
            pc.append((t, stamp(m.header), m.height * m.width, ok.sum(),
                       float(np.median(rv)) if rv.size else np.nan,
                       float(np.percentile(rv, 99)) if rv.size else np.nan,
                       float(rv.max()) if rv.size else np.nan,
                       float(iv.mean()) if iv.size else np.nan,
                       float(np.median(z[ok])) if rv.size else np.nan))
            if k % NOISE_EVERY == 0 and (m.height, m.width) == shape:
                rr = np.where(ok, rng, 0.0)
                psum += rr; psq += rr * rr; pcnt += ok
            if sample is None:   # one full frame for the 3D preview
                sel = ok.ravel()
                sample = np.stack([x.ravel()[sel], y.ravel()[sel], z.ravel()[sel],
                                   it.ravel()[sel]], 1)
            k += 1
        elif topic == IMU_T:
            m = deserialize_message(data, Imu)
            a, g = m.linear_acceleration, m.angular_velocity
            imu.append((t, stamp(m.header), a.x, a.y, a.z, g.x, g.y, g.z))
        elif topic in (T_MAIN, T_TOP):
            m = deserialize_message(data, Temperature)
            (tm if topic == T_MAIN else tt).append((t, m.temperature))
    return dict(pc=np.array(pc), imu=np.array(imu), tm=np.array(tm), tt=np.array(tt),
                shape=shape, ring_valid=ring_valid, ring_total=ring_total,
                psum=psum, psq=psq, pcnt=pcnt, sample=sample, path=path, heights=heights)


def cat(parts, key, ncol):
    arrs = [p[key] for p in parts if len(p[key])]
    a = np.concatenate(arrs) if arrs else np.zeros((0, ncol))
    return a[np.argsort(a[:, 0])] if len(a) else a


def rate_stats(t):
    d = np.diff(t)
    return dict(count=int(len(t)), hz=float((len(t) - 1) / (t[-1] - t[0])),
                period_ms_mean=float(d.mean() * 1e3), period_ms_std=float(d.std() * 1e3),
                gap_ms_max=float(d.max() * 1e3), gap_ms_p999=float(np.percentile(d, 99.9) * 1e3))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag'); ap.add_argument('out')
    ap.add_argument('--workers', type=int, default=os.cpu_count())
    args = ap.parse_args()
    files = sorted(glob.glob(os.path.join(args.bag, '*.mcap')),
                   key=lambda p: int(p.rsplit('_', 1)[1].split('.')[0]))
    with Pool(min(args.workers, len(files))) as pool:
        parts = pool.map(read_file, files)

    pc, imu = cat(parts, 'pc', 9), cat(parts, 'imu', 8)
    tm, tt = cat(parts, 'tm', 2), cat(parts, 'tt', 2)
    t0 = min(pc[0, 0], imu[0, 0])
    dur = max(pc[-1, 0], imu[-1, 0]) - t0

    # per-ring validity, per-pixel noise (merge workers)
    shp = next(p['shape'] for p in parts if p['shape'])
    ring_valid = sum(p['ring_valid'] for p in parts if p['shape'])
    ring_total = sum(p['ring_total'] for p in parts if p['shape'])
    psum = sum(p['psum'] for p in parts if p['shape'])
    psq = sum(p['psq'] for p in parts if p['shape'])
    pcnt = sum(p['pcnt'] for p in parts if p['shape'])
    n_noise = int(np.ceil(len(pc) / NOISE_EVERY))
    stable = pcnt >= 0.95 * pcnt.max()          # pixels valid in >=95 % of sampled frames
    mean = np.divide(psum, pcnt, out=np.zeros(shp), where=pcnt > 0)
    std = np.sqrt(np.maximum(np.divide(psq, pcnt, out=np.zeros(shp), where=pcnt > 0)
                             - mean ** 2, 0))
    s_std, s_mean = std[stable], mean[stable]
    # noise vs distance bands
    bands = []
    for lo, hi in ((0, 1), (1, 2), (2, 5), (5, 10), (10, 30), (30, 200)):
        sel = (s_mean >= lo) & (s_mean < hi)
        if sel.sum() > 50:
            bands.append(dict(band=f'{lo}-{hi} m', pixels=int(sel.sum()),
                              std_mm_median=float(np.median(s_std[sel]) * 1e3),
                              std_mm_p90=float(np.percentile(s_std[sel], 90) * 1e3)))

    # 10 s bins
    B = 10.0
    nb = int(np.ceil(dur / B))
    bins = []
    for i in range(nb):
        lo, hi = t0 + i * B, t0 + (i + 1) * B
        p = pc[(pc[:, 0] >= lo) & (pc[:, 0] < hi)]
        u = imu[(imu[:, 0] >= lo) & (imu[:, 0] < hi)]
        a = tm[(tm[:, 0] >= lo) & (tm[:, 0] < hi)]
        b = tt[(tt[:, 0] >= lo) & (tt[:, 0] < hi)]
        span = min(hi, t0 + dur) - lo
        f = lambda v: None if (v is None or not np.isfinite(v)) else round(float(v), 4)
        bins.append(dict(
            t=round(i * B, 1),
            pts_hz=f(len(p) / span) if span > 0 else None,
            imu_hz=f(len(u) / span) if span > 0 else None,
            pts_gap_ms=f(np.diff(p[:, 0]).max() * 1e3) if len(p) > 1 else None,
            imu_gap_ms=f(np.diff(u[:, 0]).max() * 1e3) if len(u) > 1 else None,
            valid_pct=f(100 * p[:, 3].sum() / p[:, 2].sum()) if len(p) else None,
            range_med=f(p[:, 4].mean()) if len(p) else None,
            range_p99=f(p[:, 5].mean()) if len(p) else None,
            range_max=f(p[:, 6].max()) if len(p) else None,
            intensity=f(p[:, 7].mean()) if len(p) else None,
            latency_ms=f(((p[:, 0] - p[:, 1]) * 1e3).mean()) if len(p) else None,
            acc_norm=f(np.linalg.norm(u[:, 2:5], axis=1).mean()) if len(u) else None,
            gyro_dps=[f(np.degrees(u[:, c].mean())) for c in (5, 6, 7)] if len(u) else None,
            temp_main=f(a[:, 1].mean()) if len(a) else None,
            temp_top=f(b[:, 1].mean()) if len(b) else None))

    acc, gyr = imu[:, 2:5], np.degrees(imu[:, 5:8])
    pts_stamp_d = np.diff(pc[:, 1])
    # 3D preview: downsample first frame
    smp = next(p['sample'] for p in parts if p['sample'] is not None)
    rng = np.random.default_rng(0)
    smp = smp[rng.choice(len(smp), min(20000, len(smp)), replace=False)]

    out = dict(
        bag=os.path.abspath(args.bag), files=len(files),
        size_gb=round(sum(os.path.getsize(f) for f in files) / 1e9, 2),
        start_unix=float(t0), duration_s=float(dur),
        points=dict(rate_stats(pc[:, 0]),
                    shape=list(shp), points_per_scan=int(pc[0, 2]),
                    valid_pct_mean=float(100 * pc[:, 3].sum() / pc[:, 2].sum()),
                    valid_pct_min=float(100 * (pc[:, 3] / pc[:, 2]).min()),
                    valid_pts_mean=float(pc[:, 3].mean()),
                    range_median=float(np.nanmedian(pc[:, 4])),
                    range_max=float(np.nanmax(pc[:, 6])),
                    intensity_mean=float(np.nanmean(pc[:, 7])),
                    latency_ms_mean=float(((pc[:, 0] - pc[:, 1]) * 1e3).mean()),
                    latency_ms_p99=float(np.percentile((pc[:, 0] - pc[:, 1]) * 1e3, 99)),
                    stamp_period_ms_std=float(pts_stamp_d.std() * 1e3),
                    stamp_gap_ms_max=float(pts_stamp_d.max() * 1e3),
                    expected=int(round(dur * 10)),
                    gaps_over_150ms=int((np.diff(pc[:, 0]) > 0.15).sum())),
        imu=dict(rate_stats(imu[:, 0]),
                 expected=int(round(dur * 200)),
                 gaps_over_20ms=int((np.diff(imu[:, 0]) > 0.02).sum()),
                 acc_mean=acc.mean(0).tolist(), acc_std=acc.std(0).tolist(),
                 acc_norm_mean=float(np.linalg.norm(acc, axis=1).mean()),
                 gyro_mean_dps=gyr.mean(0).tolist(), gyro_std_dps=gyr.std(0).tolist()),
        temperature=dict(
            main=dict(min=float(tm[:, 1].min()), max=float(tm[:, 1].max()),
                      mean=float(tm[:, 1].mean()), first=float(tm[0, 1]), last=float(tm[-1, 1]),
                      count=int(len(tm))),
            top=dict(min=float(tt[:, 1].min()), max=float(tt[:, 1].max()),
                     mean=float(tt[:, 1].mean()), first=float(tt[0, 1]), last=float(tt[-1, 1]),
                     count=int(len(tt)))),
        scan_columns={str(k): v for k, v in sorted(
            sum((collections.Counter(p['heights']) for p in parts), collections.Counter()).items())},
        rings=dict(valid_pct=(100 * ring_valid / ring_total).round(2).tolist()),
        noise=dict(frames_used=n_noise, stable_pixels=int(stable.sum()),
                   std_mm_median=float(np.median(s_std) * 1e3),
                   std_mm_p90=float(np.percentile(s_std, 90) * 1e3), bands=bands),
        bins=bins,
        preview=np.round(smp, 3).tolist())
    with open(args.out, 'w') as f:
        json.dump(out, f, separators=(',', ':'))
    p, u, T = out['points'], out['imu'], out['temperature']
    print(f"duration {dur/60:.1f} min, {len(files)} files, {out['size_gb']} GB")
    print(f"points {p['count']}/{p['expected']}  {p['hz']:.3f} Hz  max gap {p['gap_ms_max']:.0f} ms  "
          f"valid {p['valid_pct_mean']:.1f}%  latency {p['latency_ms_mean']:.1f} ms")
    print(f"imu    {u['count']}/{u['expected']}  {u['hz']:.2f} Hz  max gap {u['gap_ms_max']:.0f} ms  "
          f"|acc| {u['acc_norm_mean']:.3f}")
    print(f"temp   main {T['main']['min']:.2f}..{T['main']['max']:.2f}  "
          f"top {T['top']['min']:.2f}..{T['top']['max']:.2f}")
    print(f"noise  median {out['noise']['std_mm_median']:.1f} mm  p90 {out['noise']['std_mm_p90']:.1f} mm")


if __name__ == '__main__':
    main()
