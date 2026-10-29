#!/usr/bin/env python3
"""QC a ROS 2 calibration bag before you spend time on it.

Reads the bag directly (mcap or sqlite3) with rosbags -- no ROS install, no
conversion -- and reports whether what was recorded is a faithful record of what
the sensors produced.

Boundary, stated up front: this proves the *recording* is honest. It cannot tell
you whether the *sensor data* was good (focus, motion blur, exposure, whether the
AprilGrid ever filled both frames). That is what the Kalibr report is for.

Usage:
    ~/venvs/rosbags/bin/python tools/bag_qc.py ~/kalibr/w/cam_calib
    ~/venvs/rosbags/bin/python tools/bag_qc.py ~/kalibr/w/imu_cam --report
    ~/venvs/rosbags/bin/python tools/bag_qc.py --self-test

Exit codes: 0 = PASS, 1 = FAIL, 2 = deltas() not implemented / self-test failed.

See docs/kalibr_runbook.md section 3.1 for where this sits in the workflow.
"""

from __future__ import annotations

import statistics
import sys
from dataclasses import dataclass
from pathlib import Path

# --- thresholds (one place, so the verdicts are arguable) --------------------

# Expected publish rates, measured on the Pi in Phase 0.3. The IMU number is
# 202.4 rather than 200 because the BMI270's ODR sits ~1.2% above nominal.
EXPECTED_HZ = {
    '/camera/left/image_raw': 29.5,
    '/camera/right/image_raw': 29.5,
    '/imu/data_raw': 202.4,
}
SHORT = {
    '/camera/left/image_raw': 'cam_left',
    '/camera/right/image_raw': 'cam_right',
    '/imu/data_raw': 'imu',
}

# Phase 3 needs only the two images; the IMU is optional (a camera-only bag is a
# legitimate input to kalibr_calibrate_cameras) and only becomes mandatory for
# the Phase 5 camera-IMU fit.
REQUIRED_TOPICS = ('/camera/left/image_raw', '/camera/right/image_raw')

# A rate outside +/-RATE_TOL is a health flag, not a calibration verdict:
# kalibr_calibrate_cameras runs with --bag-freq 10.0, so it discards two of every
# three frames on purpose. Below RATE_FAIL_RATIO of expected, the bag is unusable.
RATE_TOL = 0.15
RATE_FAIL_RATIO = 0.50

# A delta longer than GAP_FACTOR x the median is a hole (dropped frames).
GAP_FACTOR = 3.0
HOLE_FAIL_NS = 500_000_000  # a 0.5 s hole means something really stalled

# Recorder lag = capture->written latency. A flat value is normal; growth means
# the writer is losing ground on the disk, which is the cause to fix when the gap
# counts above start going up.
LAG_DRIFT_WARN_NS = 50_000_000

EXPECTED_IMAGE = {'width': 640, 'height': 400, 'encoding': 'mono8'}
WANT_IMAGE = (f"{EXPECTED_IMAGE['width']}x{EXPECTED_IMAGE['height']} "
              f"{EXPECTED_IMAGE['encoding']} step={EXPECTED_IMAGE['width']}")

OK, WARN, FAIL = 'OK', 'WARN', 'FAIL'


# =============================================================================
# YOUR PART. Everything below this function uses its output; nothing else in
# this file needs to change.
# =============================================================================
def deltas(stamps):
    """Return consecutive differences between sorted timestamps.

    stamps:   iterable of numbers (int nanoseconds in real use, but the
              arithmetic is unit-agnostic -- it only subtracts).
    returns:  list of numbers, length len(stamps) - 1, each >= 0.

    Sort first: bag order is normally monotonic, but don't rely on it.
    A result of 0 means two messages carried the same stamp (a duplicate).

    Do not reorder or drop anything else -- the caller counts holes and
    duplicates by looking at the *values*, so the list must be 1:1 with the
    gaps between consecutive samples.
    """
    ordered = sorted(stamps)
    gaps = []
    for i in range(len(ordered)-1):
        gaps.append(ordered[i + 1] - ordered[i])
    return gaps


# --- examples that pin the behaviour down; `--self-test` runs them -----------
SELF_TESTS = [
    # The 90-second camera bag excerpt from the discussion: one 201 ms hole
    # (~6 lost frames) and one duplicate stamp, otherwise a clean 30 Hz.
    ([0, 33, 68, 101, 302, 335, 368, 401, 434, 434, 467],
     [33, 35, 33, 201, 33, 33, 33, 33, 0, 33]),
    ([5], []),
    ([], []),
    # Unsorted input: sort before differencing.
    ([3, 1, 2], [1, 1]),
]


# =============================================================================
# Everything below here is plumbing: read the bag, call deltas(), print a table.
# =============================================================================
@dataclass
class Check:
    name: str
    status: str
    detail: str


def delta_stats(stamps_ns):
    """Summary of the gap structure of one topic, or None if < 2 samples."""
    if len(stamps_ns) < 2:
        return None
    d = deltas(stamps_ns)
    ordered = sorted(d)
    med = statistics.median(d)
    return {
        'median': med,
        # max, not p99: one 200 ms hole is 1 value in 300, so it hides inside any
        # percentile. The worst hole is what you want to see.
        'max': ordered[-1],
        'gaps': sum(1 for x in d if x > GAP_FACTOR * med),
        'dups': sum(1 for x in d if x == 0),
    }


def _rate_hz(pairs):
    """(samples - 1) / span of the header stamps."""
    if len(pairs) < 2:
        return 0.0
    span = pairs[-1][1] - pairs[0][1]
    return (len(pairs) - 1) / (span / 1e9) if span > 0 else 0.0


def _human_bytes(n):
    for unit in ('B', 'kB', 'MB', 'GB'):
        if n < 1000 or unit == 'GB':
            return f'{n:.1f} {unit}' if unit != 'B' else f'{n} B'
        n /= 1000
    return f'{n:.1f} GB'


def _bag_size(path):
    if path.is_dir():
        return sum(f.stat().st_size for f in path.rglob('*') if f.is_file())
    return path.stat().st_size


def _storage(path):
    files = path.rglob('*') if path.is_dir() else [path]
    for f in files:
        if f.suffix == '.mcap':
            return 'mcap'
        if f.suffix == '.db3':
            return 'sqlite3'
    return 'unknown'


def _typestore():
    """Newest typesets this rosbags knows.

    Only a fallback: ROS 2 bags embed their own message definitions, which the
    reader prefers. LATEST rather than a pinned release so the script does not
    need editing when rosbags adds a distro.
    """
    from rosbags.typesys import Stores, get_typestore
    return get_typestore(Stores.LATEST)


def evaluate_topic(name, topic, pairs):
    """Rate + gap structure for one topic. pairs = [(bag_ts, header_stamp)]."""
    want = EXPECTED_HZ[topic]
    hz = _rate_hz(pairs)
    status, notes = OK, []
    detail = f'{hz:.1f} Hz (want ~{want:.0f})'

    if hz < want * RATE_FAIL_RATIO:
        status = FAIL
        notes.append('rate under half of expected')
    elif abs(hz / want - 1) > RATE_TOL:
        status = WARN
        notes.append('rate outside +/-15% (kalibr thins to 10 Hz anyway)')

    stats = delta_stats([s for _, s in pairs])
    if stats is None:
        notes.append(f'only {len(pairs)} message(s)')
        return Check(f'{name} data', FAIL if status == FAIL else WARN,
                     ', '.join([detail] + notes))

    detail += (f', med {stats["median"] / 1e6:.1f} ms, max {stats["max"] / 1e6:.1f} ms,'
               f' gaps {stats["gaps"]}, dup {stats["dups"]}')
    if stats['max'] > HOLE_FAIL_NS:
        status = FAIL
        notes.append('a hole that long means the recording stalled')
    elif stats['gaps'] or stats['dups']:
        if status == OK:
            status = WARN
        notes.append('frames missing or duplicated')
    return Check(f'{name} data', status, ', '.join([detail] + notes))


def inspect(bag):
    """Read the bag once and return (checks, header line)."""
    from rosbags.highlevel import AnyReader

    checks = []
    # One pass: collect (bag_timestamp, header_stamp) per topic plus the first
    # image's geometry. Connection metadata is read before iterating.
    samples = {t: [] for t in EXPECTED_HZ}
    geometry = {}
    found = []

    with AnyReader([bag], default_typestore=_typestore()) as reader:
        # reader.topics is {topic: {'msgtype', 'msgcount', 'connections'}}
        found = list(reader.topics)
        for conn, bag_ts, raw in reader.messages():
            if conn.topic not in EXPECTED_HZ:
                continue
            msg = reader.deserialize(raw, conn.msgtype)
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            samples[conn.topic].append((bag_ts, stamp))
            if conn.topic.startswith('/camera/') and conn.topic not in geometry:
                geometry[conn.topic] = (msg.width, msg.height, msg.encoding, msg.step)
        duration_s = reader.duration / 1e9

    # --- 1. topic inventory ------------------------------------------------
    missing = [t for t in REQUIRED_TOPICS if not samples.get(t)]
    unexpected = [t for t in found if t not in EXPECTED_HZ]
    if missing:
        checks.append(Check(
            'topics', FAIL,
            f'missing {", ".join(missing)} -- nothing to calibrate'))
    else:
        detail = ' '.join(t for t, pairs in samples.items() if pairs)
        if not samples.get('/imu/data_raw'):
            detail += ' (camera-only: fine for Phase 3, not for Phase 5)'
        checks.append(Check('topics', OK, detail))
    if unexpected:
        # Survivable (rosbags-convert filters by --include-topic) but it means
        # someone used `ros2 bag record -a`, which inflates the transfer.
        checks.append(Check(
            'extra topics', WARN,
            f'{", ".join(unexpected)} -- filter with --include-topic'))

    # --- 2/3. rate and gap structure, per topic ----------------------------
    for topic, pairs in samples.items():
        if pairs:
            checks.append(evaluate_topic(SHORT[topic], topic, pairs))

    # --- 4. image geometry --------------------------------------------------
    if geometry:
        bad = [f'{SHORT[t]} {w}x{h} {e} step={s}'
               for t, (w, h, e, s) in geometry.items()
               if w != EXPECTED_IMAGE['width'] or h != EXPECTED_IMAGE['height']
               or e != EXPECTED_IMAGE['encoding'] or s != w]
        if bad:
            checks.append(Check('geometry', FAIL,
                                f'{" | ".join(bad)} -- want {WANT_IMAGE}'))
        else:
            checks.append(Check('geometry', OK, f'{WANT_IMAGE}, both eyes'))

    # --- 5. stereo stamp parity --------------------------------------------
    # Both eyes are stamped from one clock read in stereo_node, so their stamps
    # must be identical. Not a Kalibr requirement -- Kalibr never assumes stereo
    # pairing -- but it is the only check here that protects OpenVINS, which does.
    left = [s for _, s in samples.get('/camera/left/image_raw', [])]
    right = [s for _, s in samples.get('/camera/right/image_raw', [])]
    if left and right:
        if len(left) != len(right):
            checks.append(Check(
                'stereo pair', FAIL,
                f'{len(left)} left vs {len(right)} right -- asymmetric drops'))
        else:
            mismatch = sum(1 for a, b in zip(left, right) if a != b)
            if mismatch:
                checks.append(Check(
                    'stereo pair', FAIL,
                    f'{mismatch}/{len(left)} stamps differ -- halves not from one frame'))
            else:
                checks.append(Check(
                    'stereo pair', OK, f'{len(left)} identical stamp pairs'))

    # --- 6. recorder lag ----------------------------------------------------
    # bag_ts - header_stamp is capture->written latency (same Pi clock). A flat
    # value is fine; a rising one means the writer is losing ground on the disk,
    # which is the cause to fix when gaps appear above.
    for topic, pairs in samples.items():
        if len(pairs) < 20:
            continue
        lags = [b - s for b, s in pairs]
        tail = max(1, len(lags) // 10)
        early = statistics.median(lags[:tail])
        late = statistics.median(lags[-tail:])
        drift = late - early
        status = WARN if abs(drift) > LAG_DRIFT_WARN_NS else OK
        detail = (f'{early / 1e6:.1f} -> {late / 1e6:.1f} ms '
                  f'(drift {drift / 1e6:+.1f} ms)')
        if status == WARN:
            detail += ' -- writer falling behind: check disk write speed'
        checks.append(Check(f'{SHORT[topic]} lag', status, detail))

    header = (f'{bag.name}  {_storage(bag)}  {_human_bytes(_bag_size(bag))}  '
              f'{duration_s:.1f} s  {len(found)} topic(s)')
    return checks, header


def overall(checks):
    statuses = {c.status for c in checks}
    if FAIL in statuses:
        return FAIL
    return WARN if WARN in statuses else OK


def render(header, checks):
    lines = [header, f'{"check":<18} {"status":<7} detail',
             f'{"-" * 18} {"-" * 7} {"-" * 60}']
    lines += [f'{c.name:<18} {c.status:<7} {c.detail}' for c in checks]
    verdict = {
        OK: 'faithful recording (says nothing about focus, blur or target coverage)',
        WARN: 'recording works; read the warnings above before trusting the result',
        FAIL: 'do not run Kalibr on this bag',
    }[overall(checks)]
    lines.append(f'{"":<18} {overall(checks):<7} {verdict}')
    return '\n'.join(lines)


def run_self_tests():
    failures = []
    for args, expected in SELF_TESTS:
        try:
            got = deltas(list(args))
        except NotImplementedError:
            return ['deltas() is not implemented yet -- that one is yours']
        except Exception as exc:  # noqa: BLE001 - surface anything, loudly
            failures.append(f'deltas({args}) raised {type(exc).__name__}: {exc}')
            continue
        if list(got) != expected:
            failures.append(f'deltas({args})\n    expected {expected}\n    got      {list(got)}')
    return failures


def main(argv):
    args = [a for a in argv[1:] if not a.startswith('-')]
    write_report = '--report' in argv

    failures = run_self_tests()
    if failures:
        print('self-test failed:\n', file=sys.stderr)
        for f in failures:
            print(f'  {f}', file=sys.stderr)
        return 2
    if '--self-test' in argv:
        print(f'self-test: {len(SELF_TESTS)}/{len(SELF_TESTS)} passed')
        return 0
    if not args:
        print(__doc__.strip().splitlines()[-1], file=sys.stderr)
        print('usage: bag_qc.py <bag_dir> [<bag_dir> ...] [--report]', file=sys.stderr)
        return 2

    worst = 0
    for arg in args:
        bag = Path(arg).expanduser()
        if not bag.exists():
            print(f'{bag}: no such file or directory', file=sys.stderr)
            worst = 1
            continue
        checks, header = inspect(bag)
        text = render(header, checks)
        print(text)
        print()
        if write_report:
            out = bag.parent / f'{bag.name}-qc.txt'
            out.write_text(text + '\n')
            print(f'  written to {out}')
        if overall(checks) == FAIL:
            worst = 1
    return worst


if __name__ == '__main__':
    sys.exit(main(sys.argv))
