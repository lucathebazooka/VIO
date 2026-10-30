#!/usr/bin/env python3
"""Cut a time window out of a rosbag2 bag, preserving message bytes and stamps.

Why this exists: Allan-variance analysis assumes the sensor sat perfectly still
for the whole sequence. The first ~30 min of a long recording is thermal
warm-up (bias drifting) and the last minutes are you walking over to stop it --
neither is stationary, and both inflate the long-cluster deviations, which is
exactly the region the bias-instability number is read from. rosbag2 has no trim
command, and `ros2 bag play --rate N` + re-record would replace the bag
timestamps (the analysis reads `header.stamp`, but a compressed replay also
mixes in dropped-message gaps). So: copy the window out, untouched.

Message payloads are copied as raw CDR bytes -- no deserialise/serialise round
trip -- so `header.stamp` is preserved bit for bit.

Usage:
    trim_bag.py SRC DST --skip 1800 --keep 37500      # 30 min in, 10h25m long
    trim_bag.py SRC DST --skip 1800 --tail 300        # 30 min in, 5 min off the end
    trim_bag.py SRC DST --skip 60 --keep 120 --topic /imu/data_raw

`SRC` is a bag directory; `DST` must not exist yet.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import StoragePlugin, Writer

NS = 1_000_000_000


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('src', type=Path, help='input bag directory')
    ap.add_argument('dst', type=Path, help='output bag directory (must not exist)')
    ap.add_argument('--skip', type=float, default=0.0,
                    help='seconds to drop from the start (default 0)')
    ap.add_argument('--keep', type=float, default=None,
                    help='seconds to keep after the skip (default: to the end)')
    ap.add_argument('--tail', type=float, default=0.0,
                    help='seconds to drop from the end (ignored with --keep)')
    ap.add_argument('--topic', action='append', default=None,
                    help='topic to keep (repeatable; default: all)')
    args = ap.parse_args(argv[1:])

    if args.dst.exists():
        print(f'{args.dst} already exists -- refusing to overwrite', file=sys.stderr)
        return 1
    if args.keep is not None and args.tail:
        print('--keep and --tail are mutually exclusive', file=sys.stderr)
        return 1

    with AnyReader([args.src]) as reader:
        conns = [c for c in reader.connections
                 if not args.topic or c.topic in args.topic]
        if not conns:
            print(f'no matching topics in {args.src}', file=sys.stderr)
            return 1

        lo = reader.start_time + int(args.skip * NS)
        if args.keep is not None:
            hi = lo + int(args.keep * NS)
        else:
            hi = reader.end_time - int(args.tail * NS)
        if hi <= lo:
            print('empty window: nothing to write', file=sys.stderr)
            return 1
        hi = min(hi, reader.end_time)

        kept = 0
        first = last = None
        with Writer(args.dst, version=9, storage_plugin=StoragePlugin.MCAP) as writer:
            out = {c.id: writer.add_connection(c.topic, c.msgtype,
                                               typestore=reader.typestore)
                   for c in conns}
            for conn, ts, raw in reader.messages(connections=conns, start=lo, stop=hi):
                writer.write(out[conn.id], ts, raw)
                kept += 1
                first = ts if first is None else first
                last = ts

    if not kept:
        print('empty window: nothing written', file=sys.stderr)
        return 1

    print(f'{args.dst}: {kept} messages, {(last - first) / NS:.1f} s '
          f'({(last - first) / NS / 3600:.2f} h), '
          f'skipped {args.skip:.0f} s at the start, '
          f'{args.tail:.0f} s at the end')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
