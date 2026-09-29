"""Part 3 -- fly the square from Part 2, but abort on a battery failsafe.

Usage: python part3_failsafe.py
Exits 0 on a clean run (mission completed OR safely aborted and landed),
non-zero on any unhandled failure/timeout.

Concurrency design: mission flying and battery monitoring are NOT run on
separate threads. pymavlink's mavutil connection reads from one TCP socket,
and recv_match() has no per-caller message queue -- if two threads both
called it on the same connection, each message would be delivered to
whichever thread's call happened to be waiting, and the other would silently
miss it. Sharing the connection safely would need a lock and a fan-out
queue, which is more moving parts than the problem calls for.

Instead this is a single-loop state machine: each iteration polls for
*either* a GLOBAL_POSITION_INT or a SYS_STATUS message (whichever arrives
next) and dispatches it to mission-progress logic or battery-watchdog logic
accordingly. Both concerns run "at the same time" in wall-clock terms --
neither can starve the other for more than one message's worth of latency --
without any locking or cross-thread state.
"""

from __future__ import annotations

import sys
import time
from datetime import datetime

from starter.drone import Drone, DroneError, horizontal_distance_m, offset_latlon

CONNECTION_STRING = "tcp:127.0.0.1:5760"
TAKEOFF_ALTITUDE_M = 15.0
SIDE_LENGTH_M = 80.0
CORNER_RADIUS_M = 2.0
BATTERY_ABORT_V = 11.0
SIM_FAILURE_INJECT_S = 20.0
SIM_FAILURE_VOLTAGE = 10.5

CORNER_OFFSETS = [
    (SIDE_LENGTH_M, 0.0),
    (SIDE_LENGTH_M, SIDE_LENGTH_M),
    (0.0, SIDE_LENGTH_M),
    (0.0, 0.0),
]

_UNKNOWN_VOLTAGE_MV = 65535  # SYS_STATUS.voltage_battery sentinel: "not reported"


def _log(mission_start: float, msg: str) -> None:
    elapsed = time.monotonic() - mission_start
    wall = datetime.now().strftime("%H:%M:%S")
    print(f"[{wall} +{elapsed:5.1f}s] {msg}")


def fly_square_with_failsafe(drone: Drone, corners: list[tuple[float, float]]) -> bool:
    """Fly the square, watching for a battery failsafe the whole time.

    Returns True if the square completed normally, False if it was aborted.
    """
    mission_start = time.monotonic()
    fault_injected = False
    aborted = False
    corner_idx = 0
    target_lat, target_lon = corners[0]
    last_goto = 0.0

    _log(mission_start, f"Mission started: flying {len(corners)}-corner square")

    while True:
        now = time.monotonic()
        elapsed = now - mission_start

        if not fault_injected and elapsed >= SIM_FAILURE_INJECT_S:
            fault_injected = True
            drone.set_param("SIM_BATT_VOLTAGE", SIM_FAILURE_VOLTAGE)
            _log(
                mission_start,
                f"Injected test failure: SIM_BATT_VOLTAGE set to "
                f"{SIM_FAILURE_VOLTAGE}V (confirmed via PARAM_VALUE)",
            )

        if not aborted and now - last_goto >= 1.0:
            last_goto = now
            drone.goto(target_lat, target_lon, TAKEOFF_ALTITUDE_M)

        msg = drone.recv_match(["GLOBAL_POSITION_INT", "SYS_STATUS"], 0.5)
        if msg is None:
            continue

        mtype = msg.get_type()

        if mtype == "SYS_STATUS" and not aborted:
            if msg.voltage_battery == _UNKNOWN_VOLTAGE_MV:
                continue
            voltage = msg.voltage_battery / 1000.0
            if voltage < BATTERY_ABORT_V:
                aborted = True
                _log(
                    mission_start,
                    f"BATTERY FAILSAFE: voltage {voltage:.2f}V dropped below "
                    f"{BATTERY_ABORT_V:.1f}V -- aborting mission, commanding RTL",
                )
                drone.set_mode("RTL")
                return False

        elif mtype == "GLOBAL_POSITION_INT" and not aborted:
            dist = horizontal_distance_m(msg.lat / 1e7, msg.lon / 1e7, target_lat, target_lon)
            if dist <= CORNER_RADIUS_M:
                _log(mission_start, f"reached corner {corner_idx + 1}/{len(corners)}")
                corner_idx += 1
                if corner_idx >= len(corners):
                    _log(mission_start, "Square complete, no failsafe triggered")
                    return True
                target_lat, target_lon = corners[corner_idx]


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
        corners = [offset_latlon(home_lat, home_lon, n, e) for n, e in CORNER_OFFSETS]

        completed = fly_square_with_failsafe(drone, corners)

        print("Waiting for RTL to land and disarm...")
        drone.rtl_and_wait()
        print("Landed and disarmed.")

        print("Mission completed normally." if completed else "Mission aborted safely on battery failsafe.")
        return 0
    except DroneError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
