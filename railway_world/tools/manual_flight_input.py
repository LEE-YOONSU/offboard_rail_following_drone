#!/usr/bin/env python3
"""Interactive MAVLink MANUAL_CONTROL sender for a PX4 SITL instance."""

import argparse
import select
import sys
import time

from pymavlink import mavutil


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=14541)
    parser.add_argument("--system", type=int, default=2)
    args = parser.parse_args()

    master = mavutil.mavlink_connection(
        f"udpin:127.0.0.1:{args.port}", source_system=250
    )
    while True:
        heartbeat = master.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
        if heartbeat and heartbeat.get_srcSystem() == args.system:
            break

    target = args.system
    controls = [0, 0, 500, 0]
    command_until = 0.0
    print("READY: enter 'x y z r seconds', MODE POSCTL, or QUIT", flush=True)

    while True:
        now = time.monotonic()
        if command_until and now >= command_until:
            controls = [0, 0, 500, 0]
            command_until = 0.0
            print("NEUTRAL", flush=True)

        master.mav.manual_control_send(target, *controls, 0)

        # Headless Gazebo can run much faster than wall time. Keep manual input
        # fresh in simulation time so PX4 does not interpret it as RC loss.
        ready, _, _ = select.select([sys.stdin], [], [], 0.002)
        if not ready:
            continue
        line = sys.stdin.readline()
        if not line:
            break
        fields = line.strip().split()
        if not fields:
            continue
        if fields[0].upper() == "QUIT":
            break
        if fields[0].upper() == "MODE" and len(fields) == 2:
            master.set_mode(fields[1].upper())
            print(f"MODE {fields[1].upper()}", flush=True)
            continue
        if len(fields) != 5:
            print("ERROR: expected x y z r seconds", flush=True)
            continue
        controls = [int(value) for value in fields[:4]]
        command_until = time.monotonic() + float(fields[4])
        print(f"INPUT {controls} for {fields[4]} s", flush=True)

    for _ in range(20):
        master.mav.manual_control_send(target, 0, 0, 500, 0, 0)
        time.sleep(0.05)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
