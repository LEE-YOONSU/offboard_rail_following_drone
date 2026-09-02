#!/usr/bin/env python3
"""Save frames from the Gazebo /overview/image topic until stopped (SIGINT/TERM)."""
from __future__ import annotations

import signal
import sys
import time
from pathlib import Path

import numpy as np
from gz.msgs10.image_pb2 import Image as GzImage
from gz.transport13 import Node
from PIL import Image as PilImage

OUT = Path(__file__).resolve().parents[1] / "output" / "overview_frames"
TOPIC = "/overview/image"


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    for old in OUT.glob("*.jpg"):
        old.unlink()

    state = {"i": 0, "last": 0.0, "run": True}

    def cb(msg: GzImage) -> None:
        expected = msg.width * msg.height * 3
        if msg.width <= 0 or len(msg.data) < expected:
            return
        now = time.monotonic()
        if now - state["last"] < 1.0 / 15:
            return
        state["last"] = now
        frame = np.frombuffer(msg.data[:expected], dtype=np.uint8).reshape(
            msg.height, msg.width, 3
        )
        PilImage.fromarray(frame, "RGB").save(OUT / f"f_{state['i']:06d}.jpg", quality=85)
        state["i"] += 1

    node = Node()
    node.subscribe(GzImage, TOPIC, cb)

    def stop(_s, _f):
        state["run"] = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    print(f"grab_overview: recording {TOPIC} -> {OUT}", flush=True)
    while state["run"]:
        time.sleep(0.2)
    print(f"grab_overview: saved {state['i']} frames", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
