#!/usr/bin/env python3
"""Save Gazebo camera messages as PNG frames plus JSONL timestamps."""

import argparse
import json
import threading
import time
from pathlib import Path

from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node
from PIL import Image as PilImage


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/x500/rail_down_camera/image")
    parser.add_argument("--output", required=True)
    parser.add_argument("--fps", type=float, default=5.0)
    parser.add_argument("--duration", type=float, default=20.0)
    args = parser.parse_args()

    output = Path(args.output)
    frames = output / "rgb"
    frames.mkdir(parents=True, exist_ok=True)
    metadata_path = output / "frames.jsonl"
    lock = threading.Lock()
    last_saved = 0.0
    frame_index = 0

    metadata = metadata_path.open("w", encoding="utf-8")

    def on_image(msg: Image) -> None:
        nonlocal last_saved, frame_index
        now = time.monotonic()
        with lock:
            if now - last_saved < 1.0 / args.fps:
                return
            last_saved = now
            index = frame_index
            frame_index += 1

        expected = msg.width * msg.height * 3
        if len(msg.data) < expected:
            return

        filename = f"frame_{index:06d}.png"
        image = PilImage.frombytes("RGB", (msg.width, msg.height), msg.data[:expected])
        image.save(frames / filename, compress_level=3)

        stamp = msg.header.stamp
        record = {
            "frame": index,
            "file": f"rgb/{filename}",
            "sim_time_sec": stamp.sec + stamp.nsec / 1_000_000_000.0,
            "width": msg.width,
            "height": msg.height,
            "topic": args.topic,
        }
        metadata.write(json.dumps(record, ensure_ascii=False) + "\n")
        metadata.flush()

    node = Node()
    node.subscribe(Image, args.topic, on_image)
    print(f"Capturing {args.topic} to {output} at up to {args.fps:g} FPS", flush=True)
    time.sleep(args.duration)
    node.unsubscribe(args.topic)
    metadata.close()
    print(f"Saved {frame_index} frames", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
