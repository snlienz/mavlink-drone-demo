"""Part 2 -- take off, fly an 80m square, then RTL and wait for landing.

Usage: python part2_square.py
Exits 0 on success, non-zero on any failure/timeout.
"""

from __future__ import annotations

import sys

from starter.drone import Drone, DroneError, offset_latlon

CONNECTION_STRING = "tcp:127.0.0.1:5760"
TAKEOFF_ALTITUDE_M = 15.0
SIDE_LENGTH_M = 80.0
CORNER_RADIUS_M = 2.0

# Square corners as (north, east) meters offset from home, visited in order,
# ending back at the start corner.
CORNER_OFFSETS = [
    (SIDE_LENGTH_M, 0.0),
    (SIDE_LENGTH_M, SIDE_LENGTH_M),
    (0.0, SIDE_LENGTH_M),
    (0.0, 0.0),
]


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

        home_lat, home_lon, _ = drone.current_position()
        print(f"Home position: lat={home_lat:.7f} lon={home_lon:.7f}")

        for i, (north, east) in enumerate(CORNER_OFFSETS, start=1):
            lat, lon = offset_latlon(home_lat, home_lon, north, east)
            print(f"Flying to corner {i}/{len(CORNER_OFFSETS)} "
                  f"(north={north:.0f}m east={east:.0f}m)...")
            drone.fly_to(lat, lon, TAKEOFF_ALTITUDE_M, reach_radius_m=CORNER_RADIUS_M)
            print(f"  reached corner {i}")

        print("Square complete. Commanding RTL and waiting for landing...")
        drone.rtl_and_wait()
        print("Landed and disarmed.")

        return 0
    except DroneError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
