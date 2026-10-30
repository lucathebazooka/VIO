#!/usr/bin/env python3
"""Compare how sharp the two eyes of a stereo bag are.

Both eyes come off one sensor but through two lenses, which in a cheap module are
focused separately (and can have different amounts of dust/glare). An asymmetric
reprojection residual between the two cameras -- Kalibr reports one per camera --
is usually that, not a model or motion problem.

The bag stores decoded mono8 images (stereo_node decodes before publishing), so
this needs only numpy + rosbags: no OpenCV.

Per sampled frame it computes, per eye:

  grad      mean |dI/dx| + |dI/dy|      raw edge strength (scale-dependent)
  lap_var   variance of the Laplacian   classic autofocus metric (scale-dependent)
  sharp     grad / std(I)               contrast-normalised edge strength
  hf        fraction of FFT energy above 1/4 of Nyquist   (inherently normalised)

`grad`/`lap_var` rise with contrast, so a darker eye looks softer for free; `sharp`
and `hf` are the ones to compare across eyes. The ratio is reported per frame
(right/left) and as a median, because scene content differs slightly between the
halves and the pairing cancels most of that.

Usage:
    ~/venvs/rosbags/bin/python tools/eye_sharpness.py ~/kalibr/w/cam_calib [n_frames]
"""

from __future__ import annotations

import statistics
import sys
from pathlib import Path

import numpy as np

LEFT = '/camera/left/image_raw'
RIGHT = '/camera/right/image_raw'


def metrics(img: np.ndarray) -> dict[str, float]:
    a = img.astype(np.float32)
    gy, gx = np.diff(a, axis=0), np.diff(a, axis=1)
    grad = (np.abs(gy).mean() + np.abs(gx).mean()) / 2
    lap = (-4 * a[1:-1, 1:-1] + a[:-2, 1:-1] + a[2:, 1:-1]
           + a[1:-1, :-2] + a[1:-1, 2:])
    sd = float(a.std()) or 1e-6

    spec = np.abs(np.fft.rfft2(a - a.mean())) ** 2
    fy = np.fft.fftfreq(a.shape[0])[:, None]
    fx = np.fft.rfftfreq(a.shape[1])[None, :]
    radius = np.sqrt(fy ** 2 + fx ** 2)
    total = spec.sum() or 1e-9
    hf = spec[radius > 0.25].sum() / total

    return {'grad': float(grad), 'lap_var': float(lap.var()),
            'sharp': float(grad) / sd, 'hf': float(hf), 'mean': float(a.mean())}


def main(argv):
    from rosbags.highlevel import AnyReader

    bag = Path(argv[1]).expanduser()
    want = int(argv[2]) if len(argv) > 2 else 60

    frames: dict[str, list[np.ndarray]] = {LEFT: [], RIGHT: []}
    with AnyReader([bag]) as reader:
        conns = [c for c in reader.connections if c.topic in frames]
        total = max(c.msgcount for c in conns)
        step = max(1, total // want)
        counts = {LEFT: 0, RIGHT: 0}
        for conn, _ts, raw in reader.messages(connections=conns):
            i = counts[conn.topic]
            counts[conn.topic] += 1
            if i % step:
                continue
            msg = reader.deserialize(raw, conn.msgtype)
            data = np.asarray(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
            frames[conn.topic].append(data)
            if len(frames[LEFT]) >= want and len(frames[RIGHT]) >= want:
                break

    n = min(len(frames[LEFT]), len(frames[RIGHT]))
    if n == 0:
        print(f'{bag}: no stereo pairs found', file=sys.stderr)
        return 2

    keys = ('grad', 'lap_var', 'sharp', 'hf', 'mean')
    per_eye = {t: {k: [] for k in keys} for t in frames}
    ratios = {k: [] for k in ('grad', 'lap_var', 'sharp', 'hf')}

    for i in range(n):
        ml = metrics(frames[LEFT][i])
        mr = metrics(frames[RIGHT][i])
        for k in keys:
            per_eye[LEFT][k].append(ml[k])
            per_eye[RIGHT][k].append(mr[k])
        for k in ratios:
            if ml[k]:
                ratios[k].append(mr[k] / ml[k])

    print(f'{bag.name}  {n} paired frames\n')
    print(f'{"metric":9} {"left":>10} {"right":>10} {"right/left":>11}   meaning')
    meaning = {
        'grad': 'raw edge strength (follows contrast)',
        'lap_var': 'autofocus sharpness (follows contrast)',
        'sharp': 'edge strength / contrast  <- compare this',
        'hf': 'share of energy above 1/4 Nyquist  <- and this',
        'mean': 'mean brightness (exposure, not focus)',
    }
    for k in keys:
        l = statistics.median(per_eye[LEFT][k])
        r = statistics.median(per_eye[RIGHT][k])
        if k in ratios:
            med = statistics.median(ratios[k])
            print(f'{k:9} {l:10.4f} {r:10.4f} {med:11.3f}   {meaning[k]}')
        else:
            print(f'{k:9} {l:10.1f} {r:10.1f} {"-":>11}   {meaning[k]}')

    for k in ('sharp', 'hf'):
        med = statistics.median(ratios[k])
        lo, hi = np.percentile(ratios[k], [10, 90])
        print(f'\n{k}: right/left median {med:.3f} (p10-p90 {lo:.3f}-{hi:.3f})')
        if 0.9 <= med <= 1.11:
            verdict = 'comparable'
        elif med < 0.9:
            verdict = 'RIGHT is softer'
        else:
            verdict = 'LEFT is softer'
        print(f'  -> {verdict}')

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
