"""Part 1 -- connect, wait for a position estimate, arm, and take off to 15m.

Usage: python part1_takeoff.py
Exits 0 on success ("REACHED" printed), non-zero on any failure/timeout.
"""

from __future__ import annotations

import sys

from starter.drone import Drone, DroneError

CONNECTION_STRING = "tcp:127.0.0.1:5760"
TAKEOFF_ALTITUDE_M = 15.0


def main() -> int:
    try:
        print(f"Connecting to {CONNECTION_STRING} ...")
        drone = Drone(CONNECTION_STRING)
        print("Connected. Waiting for EKF position estimate (~40-60s after boot)...")
        drone.wait_until_ready()

        print("Switching to GUIDED...")
        drone.set_mode("GUIDED")

        print("Arming...")
        drone.arm()

        print(f"Taking off to {TAKEOFF_ALTITUDE_M}m...")
        drone.takeoff(TAKEOFF_ALTITUDE_M)

        return 0
    except DroneError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
