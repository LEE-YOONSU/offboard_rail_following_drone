#!/usr/bin/env python3
"""Capture one downward-camera keyframe at every main-track module."""

import argparse
import json
import math
import subprocess
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node
from PIL import Image as PilImage


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--height", type=float, default=3.112)
    args = parser.parse_args()

    world_path = Path(args.world)
    output = Path(args.output)
    rgb_dir = output / "rgb"
    rgb_dir.mkdir(parents=True, exist_ok=True)

    root = ET.parse(world_path).getroot()
    track_poses = []
    for include in root.findall(".//world/include"):
        name = include.findtext("name", "")
        if not name.startswith("track_"):
            continue
        values = [float(value) for value in include.findtext("pose").split()]
        track_poses.append((name, values[0], values[1], values[5]))

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
    for index, (name, x, y, yaw) in enumerate(track_poses):
        half_yaw = yaw * 0.5
        request = (
            f'name: "x500" position {{ x: {x} y: {y} z: {args.height} }} '
            f'orientation {{ z: {math.sin(half_yaw)} w: {math.cos(half_yaw)} }}'
        )
        subprocess.run(
            [
                "gz", "service", "-s", "/world/railway_environment/set_pose",
                "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
                "--timeout", "5000", "--req", request,
            ],
            check=True,
            stdout=subprocess.DEVNULL,
        )

        # Ogre's sensor render queue can still contain frames from the previous
        # pose. Discard several updates before taking the position-synchronised
        # keyframe.
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

        filename = f"{index:03d}_{name}.png"
        expected = msg.width * msg.height * 3
        PilImage.frombytes("RGB", (msg.width, msg.height), msg.data[:expected]).save(
            rgb_dir / filename, compress_level=3
        )
        records.append({
            "index": index, "track_module": name, "file": f"rgb/{filename}",
            "pose": {"x": x, "y": y, "z": args.height, "yaw": yaw},
            "sim_time_sec": msg.header.stamp.sec + msg.header.stamp.nsec / 1e9,
        })
        print(f"[{index + 1}/{len(track_poses)}] {name}", flush=True)

    (output / "frames.jsonl").write_text(
        "".join(json.dumps(record) + "\n" for record in records), encoding="utf-8"
    )
    print(f"Saved {len(records)} track keyframes to {output}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
