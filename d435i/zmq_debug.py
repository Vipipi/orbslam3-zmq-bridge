#!/usr/bin/env python3
import argparse
import signal
import struct
import sys

from typing import List

import numpy as np
import zmq


def parse_u64_le(b: bytes) -> int:
    return struct.unpack('<Q', b)[0]


def parse_mat4f_le(b: bytes) -> np.ndarray:
    arr = np.frombuffer(b, dtype='<f4')
    if arr.size != 16:
        raise ValueError('Expected 16 float32s for 4x4 matrix')
    return arr.reshape(4, 4)


def normalize_topics(topics: List[str]) -> List[bytes]:
    out: List[bytes] = []
    for t in topics:
        parts = [p for p in (t.split(',') if ',' in t else [t]) if p != '']
        for p in parts:
            out.append(p.encode('ascii', errors='ignore'))
    return out


def main():
    ap = argparse.ArgumentParser(description='Subscribe and print SLAM ZMQ topics')
    ap.add_argument('-e', '--endpoint', action='append', required=True,
                    help='tcp://host:port of bridge PUB endpoint; repeat to add more')
    ap.add_argument('-t', '--topic', action='append', default=None,
                    help='Topic prefix to subscribe; repeatable. If omitted, subscribes to all')
    args = ap.parse_args()

    ctx = zmq.Context.instance()
    sub = ctx.socket(zmq.SUB)
    for ep in args.endpoint:
        sub.connect(ep)

    # Subscribe to all if no topics specified, else to provided prefixes
    if not args.topic:
        sub.setsockopt(zmq.SUBSCRIBE, b'')
        sel_desc = '(all topics)'
    else:
        topics = normalize_topics(args.topic)
        if not topics:
            sub.setsockopt(zmq.SUBSCRIBE, b'')
            sel_desc = '(all topics)'
        else:
            for t in topics:
                sub.setsockopt(zmq.SUBSCRIBE, t)
            sel_desc = ', '.join(t.decode('ascii', errors='ignore') for t in topics)

    print(f"Subscribed to {', '.join(args.endpoint)} topics: {sel_desc}")

    stop = False

    def _sigint(_sig, _frm):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _sigint)
    signal.signal(signal.SIGTERM, _sigint)

    while not stop:
        try:
            frames = sub.recv_multipart()
        except KeyboardInterrupt:
            break

        if not frames:
            continue

        try:
            topic = frames[0].decode('ascii', errors='replace')
        except Exception:
            topic = '<decode_error>'

        # Known topics with structured payloads
        if topic == 'slam/tracking_pose' and len(frames) >= 3:
            try:
                t_ns = parse_u64_le(frames[1])
                Twc = parse_mat4f_le(frames[2])
                print(f"[tracking] t={t_ns} ns\n{Twc}")
            except Exception as exc:
                print(f"[tracking] parse error: {exc} (frames={[len(f) for f in frames[1:]]})")
        elif topic == 'slam/kf_pose' and len(frames) >= 4:
            try:
                kf_id = parse_u64_le(frames[1])
                t_ns = parse_u64_le(frames[2])
                Twc = parse_mat4f_le(frames[3])
                print(f"[kf] id={kf_id} t={t_ns} ns\n{Twc}")
            except Exception as exc:
                print(f"[kf] parse error: {exc} (frames={[len(f) for f in frames[1:]]})")
        elif topic == '/slam/kf_pose_update' and len(frames) >= 4:
            try:
                map_id = parse_u64_le(frames[1])
                kf_id = parse_u64_le(frames[2])
                Twc = parse_mat4f_le(frames[3])
                print(f"[/kf_pose_update] map={map_id} kf={kf_id}\n{Twc}")
            except Exception as exc:
                print(f"[/kf_pose_update] parse error: {exc} (frames={[len(f) for f in frames[1:]]})")
        elif topic == '/slam/map_new' and len(frames) >= 3:
            try:
                map_id = parse_u64_le(frames[1])
                created_ns = parse_u64_le(frames[2])
                print(f"[/map_new] map={map_id} created_at={created_ns} ns")
            except Exception as exc:
                print(f"[/map_new] parse error: {exc} (frames={[len(f) for f in frames[1:]]})")
        elif topic == '/slam/map_switch' and len(frames) >= 4:
            try:
                from_id = parse_u64_le(frames[1])
                to_id = parse_u64_le(frames[2])
                reason = frames[3].decode('ascii', errors='replace')
                print(f"[/map_switch] {from_id} -> {to_id} reason='{reason}'")
            except Exception as exc:
                print(f"[/map_switch] parse error: {exc} (frames={[len(f) for f in frames[1:]]})")
        else:
            sizes = [len(f) for f in frames[1:]]
            print(f"[{topic}] payload frames={len(frames)-1} sizes={sizes}")

    print('Exiting.')


if __name__ == '__main__':
    main()


