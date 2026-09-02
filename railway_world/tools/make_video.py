#!/usr/bin/env python3
"""Assemble a directory of f_*.jpg frames into an .mp4 (cv2, mp4v codec)."""
from __future__ import annotations

import sys
from pathlib import Path

import cv2


def main() -> int:
    if len(sys.argv) not in (3, 4):
        print("usage: make_video.py <frames_dir> <out.mp4> [fps]", file=sys.stderr)
        return 2
    frames_dir = Path(sys.argv[1])
    out_path = Path(sys.argv[2])
    fps = float(sys.argv[3]) if len(sys.argv) > 3 else 15.0

    frames = sorted(frames_dir.glob("f_*.jpg"))
    if not frames:
        print(f"no frames in {frames_dir}", file=sys.stderr)
        return 1

    first = cv2.imread(str(frames[0]))
    h, w = first.shape[:2]
    vw = cv2.VideoWriter(str(out_path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
    if not vw.isOpened():
        print("VideoWriter failed to open", file=sys.stderr)
        return 1
    for f in frames:
        img = cv2.imread(str(f))
        if img is None:
            continue
        if img.shape[:2] != (h, w):
            img = cv2.resize(img, (w, h))
        vw.write(img)
    vw.release()
    print(f"wrote {out_path}  ({len(frames)} frames @ {fps}fps -> {len(frames)/fps:.1f}s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
