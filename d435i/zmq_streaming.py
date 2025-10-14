#!/usr/bin/env python3
import argparse
import signal
import sys
import time
import struct

import numpy as np
import cv2
import zmq

try:
    import pyrealsense2 as rs
except Exception as exc:
    print("pyrealsense2 not found. Install with: python -m pip install pyrealsense2", file=sys.stderr)
    raise


def le_u64(v: int) -> bytes:
    return struct.pack('<Q', int(v))


def main():
    ap = argparse.ArgumentParser(description="Publish D435/D435i RGB-D frames to ORB-SLAM3 ZMQ bridge")
    ap.add_argument('--endpoint', required=True, help='tcp://host:port to connect (bridge SUB endpoint)')
    ap.add_argument('--topic', default='rgbd/input')
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--jpeg-quality', type=int, default=90)
    ap.add_argument('--serial', default='', help='Optional device serial')
    args = ap.parse_args()

    # ZMQ publisher connects to the bridge's bound SUB socket
    ctx = zmq.Context.instance()
    pub = ctx.socket(zmq.PUB)
    pub.connect(args.endpoint)

    # RealSense pipeline setup
    pipeline = rs.pipeline()
    config = rs.config()
    if args.serial:
        config.enable_device(args.serial)
    config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
    config.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)

    align_to_color = rs.align(rs.stream.color)
    profile = pipeline.start(config)

    # Graceful shutdown support
    stop = False
    def _sigint(_sig, _frm):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGINT, _sigint)
    signal.signal(signal.SIGTERM, _sigint)

    try:
        print(f"Publishing D435 RGB-D to {args.endpoint} on topic '{args.topic}' at {args.fps} FPS …")
        quality_param = [int(cv2.IMWRITE_JPEG_QUALITY), int(args.jpeg-quality) if hasattr(args, 'jpeg-quality') else int(args.jpeg_quality)]
        # Fallback for argparse dest name
        try:
            jpeg_quality = int(args.jpeg_quality)
        except AttributeError:
            jpeg_quality = int(getattr(args, 'jpeg-quality'))
        quality_param = [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality]

        period = 1.0 / float(args.fps) if args.fps > 0 else 0.0
        while not stop:
            frames = pipeline.wait_for_frames()
            frames = align_to_color.process(frames)

            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Convert to numpy arrays
            color = np.asanyarray(color_frame.get_data())  # HxWx3 BGR
            depth = np.asanyarray(depth_frame.get_data())  # HxW uint16 z16 (sensor scale typically 0.001 m per unit)

            # JPEG encode color
            ok, jpg = cv2.imencode('.jpg', color, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])
            if not ok:
                continue

            # Timestamp in nanoseconds (host monotonic time)
            t_ns = time.time_ns()

            # Send multipart message: [topic][t_ns][jpeg][depth_raw]
            pub.send_multipart([
                args.topic.encode('ascii'),
                le_u64(t_ns),
                jpg.tobytes(),
                depth.tobytes(),
            ])

            if period > 0:
                # coarse pacing; bridge processes as fast as it can regardless
                dt = period - (time.time_ns() - t_ns) * 1e-9
                if dt > 0:
                    time.sleep(dt)
    finally:
        try:
            pipeline.stop()
        except Exception:
            pass


if __name__ == '__main__':
    main()


