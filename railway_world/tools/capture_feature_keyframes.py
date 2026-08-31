#!/usr/bin/env python3
"""Capture synchronized downward-camera keyframes at important railway features."""

import argparse
import json
import math
import subprocess
import threading
import time
from pathlib import Path

from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node
from PIL import Image as PilImage


FEATURES = (
    ("turnout", 79.5066, 7.5809, 0.195),
    ("branch_mid", 102.74, 8.61, -0.018),
    ("branch_end", 133.58, 7.25, -0.05),
    ("curve_mid", 137.2787, 23.5698, 0.345),
    ("level_crossing", 175.9862, 39.8212, 0.45),
    ("curve_terminal", 231.1470, 72.1408, 0.61),
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--height", type=float, required=True,
                        help="Camera world Z in metres (rail top is Z=0.612 m)")
    args = parser.parse_args()

    output = Path(args.output)
    rgb_dir = output / "rgb"
    rgb_dir.mkdir(parents=True, exist_ok=True)
    latest = {"msg": None, "sequence": 0}
    lock = threading.Lock()

    def on_image(msg: Image) -> None:
        with lock:
            latest["msg"] = msg
            latest["sequence"] += 1

    node = Node()
    topic = "/x500/rail_down_camera/image"
    node.subscribe(Image, topic, on_image)
    records = []

    for index, (name, x, y, yaw) in enumerate(FEATURES):
        request = (
            f'name: "x500" position {{ x: {x} y: {y} z: {args.height} }} '
            f'orientation {{ z: {math.sin(yaw / 2)} w: {math.cos(yaw / 2)} }}'
        )
        subprocess.run(
            ["gz", "service", "-s", "/world/railway_environment/set_pose",
             "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
             "--timeout", "5000", "--req", request],
            check=True, stdout=subprocess.DEVNULL,
        )
        with lock:
            before = latest["sequence"]
        deadline = time.monotonic() + 3.0
        msg = None
        while time.monotonic() < deadline:
            time.sleep(0.05)
            with lock:
                advanced = latest["sequence"] - before
                msg = latest["msg"]
            if advanced >= 6:
                break
        if msg is None:
            raise RuntimeError(f"No camera frame received at {name}")

        filename = f"{index:02d}_{name}.png"
        expected = msg.width * msg.height * 3
        PilImage.frombytes("RGB", (msg.width, msg.height), msg.data[:expected]).save(
            rgb_dir / filename, compress_level=3
        )
        records.append({
            "index": index, "feature": name, "file": f"rgb/{filename}",
            "pose": {"x": x, "y": y, "z": args.height, "yaw": yaw},
            "height_above_rail_m": round(args.height - 0.612, 3),
            "sim_time_sec": msg.header.stamp.sec + msg.header.stamp.nsec / 1e9,
        })
        print(f"[{index + 1}/{len(FEATURES)}] {name}", flush=True)

    (output / "frames.jsonl").write_text(
        "".join(json.dumps(record) + "\n" for record in records), encoding="utf-8"
    )
    print(f"Saved {len(records)} feature keyframes to {output}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
