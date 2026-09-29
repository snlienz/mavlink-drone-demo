"""Drone controller: connect, arm, take off, and (later) navigate.

Handles the MAVLink plumbing: connecting, heartbeats, telemetry streams,
mode switching, arming, and takeoff. Every blocking call confirms its
outcome via ack and/or telemetry, with a timeout -- no blind sleeps.
"""

from __future__ import annotations

import math
import time

from pymavlink import mavutil


class DroneError(Exception):
    """Raised when the drone rejects a command or a step times out."""


# Bits in EKF_STATUS_REPORT.flags that indicate a usable position estimate
# for a GUIDED takeoff (absolute position good or about to be good).
_EKF_POSITION_OK = (
    mavutil.mavlink.EKF_POS_HORIZ_ABS | mavutil.mavlink.EKF_PRED_POS_HORIZ_ABS
)

# Only used for local flat-earth offsets over distances of tens/hundreds of
# meters (a square's worth of flight) -- not meant for long-range navigation.
_EARTH_RADIUS_M = 6378137.0


def offset_latlon(lat: float, lon: float, north_m: float, east_m: float) -> tuple[float, float]:
    """Return the (lat, lon) reached by moving north_m/east_m meters from (lat, lon)."""
    d_lat = north_m / _EARTH_RADIUS_M
    d_lon = east_m / (_EARTH_RADIUS_M * math.cos(math.radians(lat)))
    return lat + math.degrees(d_lat), lon + math.degrees(d_lon)


def horizontal_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Approximate ground distance in meters between two nearby lat/lon points."""
    north = math.radians(lat2 - lat1) * _EARTH_RADIUS_M
    east = math.radians(lon2 - lon1) * _EARTH_RADIUS_M * math.cos(math.radians(lat1))
    return math.hypot(north, east)


class Drone:
    def __init__(self, connection_string: str = "tcp:127.0.0.1:5760") -> None:
        try:
            self.conn = mavutil.mavlink_connection(connection_string)
            if self.conn.wait_heartbeat(timeout=30) is None:
                raise DroneError("no heartbeat — is SITL running?")
            self.conn.mav.request_data_stream_send(
                self.conn.target_system,
                self.conn.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                4,
                1,
            )
        except OSError as exc:
            raise DroneError(f"could not connect to {connection_string}: {exc}") from exc

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def recv_match(self, type, timeout: float):
        """conn.recv_match, but a dead connection raises DroneError.

        A closed/reset TCP socket doesn't always raise cleanly -- pymavlink's
        non-autoreconnecting TCP driver can instead return no data forever
        (spamming "EOF on TCP socket") and only raise later once the OS
        fully tears the connection down. Either way this must end up as a
        clean DroneError, never a raw socket traceback out of a flight loop.
        The short sleep on a miss also bounds the loop rate during that EOF
        spam, since a closed socket makes select() return immediately.
        """
        try:
            msg = self.conn.recv_match(type=type, blocking=True, timeout=timeout)
        except OSError as exc:
            raise DroneError(f"lost connection to drone: {exc}") from exc
        if msg is None:
            time.sleep(0.05)
        return msg

    def _send_command(
        self,
        command: int,
        params: tuple[float, float, float, float, float, float, float],
        timeout: float,
    ) -> None:
        """Send a COMMAND_LONG and block until it is accepted.

        Raises DroneError if the command is rejected or no ack arrives
        within `timeout`.
        """
        try:
            self.conn.mav.command_long_send(
                self.conn.target_system,
                self.conn.target_component,
                command,
                0,  # confirmation
                *params,
            )
        except OSError as exc:
            raise DroneError(f"lost connection to drone: {exc}") from exc
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            msg = self.recv_match("COMMAND_ACK", max(remaining, 0))
            if msg is None:
                break
            if msg.command != command:
                continue
            if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                return
            if msg.result == mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
                # Keep waiting for the final result.
                continue
            raise DroneError(
                f"command {command} rejected: result={msg.result}"
            )
        raise DroneError(f"command {command} timed out waiting for ack")

    # ------------------------------------------------------------------
    # Flight operations
    # ------------------------------------------------------------------

    def wait_until_ready(self, timeout: float = 90.0) -> None:
        """Block until the EKF has a good enough position estimate to fly.

        A guided takeoff needs a converged position estimate, which takes
        ~40-60 s after boot while the simulated GPS/EKF settle. We watch
        EKF_STATUS_REPORT for the relevant flags, and print STATUSTEXT
        messages (e.g. "Need Position Estimate") along the way so the
        wait isn't a silent black box.
        """
        deadline = time.monotonic() + timeout
        last_status_print = 0.0
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            msg = self.recv_match(["EKF_STATUS_REPORT", "STATUSTEXT"], max(remaining, 0))
            if msg is None:
                break
            mtype = msg.get_type()
            if mtype == "STATUSTEXT":
                now = time.monotonic()
                if now - last_status_print > 2.0:
                    last_status_print = now
                    print(f"  [autopilot] {msg.text}")
            elif mtype == "EKF_STATUS_REPORT":
                if msg.flags & _EKF_POSITION_OK == _EKF_POSITION_OK:
                    return
        raise DroneError("timed out waiting for EKF position estimate")

    def set_mode(self, mode_name: str, timeout: float = 10.0) -> None:
        """Switch flight mode and confirm the switch happened."""
        mode_map = self.conn.mode_mapping()
        if mode_map is None or mode_name not in mode_map:
            raise DroneError(f"unknown mode '{mode_name}'")
        custom_mode = mode_map[mode_name]

        self._send_command(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            (mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, custom_mode, 0, 0, 0, 0, 0),
            timeout,
        )

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            msg = self.recv_match("HEARTBEAT", max(remaining, 0))
            if msg is None:
                break
            if msg.custom_mode == custom_mode:
                return
        raise DroneError(f"mode change to '{mode_name}' not confirmed by telemetry")

    def arm(self, timeout: float = 30.0) -> None:
        """Arm motors. Confirm via ack and telemetry."""
        self._send_command(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            (1, 0, 0, 0, 0, 0, 0),
            timeout,
        )

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            msg = self.recv_match("HEARTBEAT", max(remaining, 0))
            if msg is None:
                break
            if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                return
        raise DroneError("arm command acked but motors never showed armed")

    def takeoff(self, altitude_m: float, timeout: float = 60.0) -> None:
        """Take off and block until within 0.5 m of the target altitude."""
        self._send_command(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            (0, 0, 0, 0, 0, 0, altitude_m),
            timeout,
        )

        deadline = time.monotonic() + timeout
        last_print = 0.0
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            msg = self.recv_match("GLOBAL_POSITION_INT", max(remaining, 0))
            if msg is None:
                break
            alt = msg.relative_alt / 1000.0
            now = time.monotonic()
            if now - last_print > 1.0:
                last_print = now
                print(f"  alt={alt:.1f}m")
            if abs(altitude_m - alt) <= 0.5:
                print("REACHED")
                return
        raise DroneError(
            f"takeoff timed out before reaching {altitude_m}m (target altitude not reached)"
        )

    def current_position(self, timeout: float = 5.0) -> tuple[float, float, float]:
        """Return the current (lat, lon, relative_alt_m) from telemetry."""
        msg = self.recv_match("GLOBAL_POSITION_INT", timeout)
        if msg is None:
            raise DroneError("no GLOBAL_POSITION_INT received")
        return msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0

    def goto(self, lat: float, lon: float, alt_m: float) -> None:
        """Send a single position setpoint (GUIDED mode). Does not block."""
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        try:
            self.conn.mav.set_position_target_global_int_send(
                0,  # time_boot_ms (unused by ArduPilot for this message)
                self.conn.target_system,
                self.conn.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                type_mask,
                int(lat * 1e7),
                int(lon * 1e7),
                alt_m,
                0, 0, 0,
                0, 0, 0,
                0, 0,
            )
        except OSError as exc:
            raise DroneError(f"lost connection to drone: {exc}") from exc

    def fly_to(
        self,
        lat: float,
        lon: float,
        alt_m: float,
        reach_radius_m: float = 2.0,
        timeout: float = 120.0,
        resend_interval: float = 1.0,
    ) -> None:
        """Fly to a lat/lon/alt and block until within reach_radius_m horizontally.

        Position setpoints are resent periodically: ArduPilot holds the last
        GUIDED target, but a stale target is a needless risk on a multi-leg
        mission, and resending costs nothing.
        """
        deadline = time.monotonic() + timeout
        last_send = 0.0
        last_print = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now - last_send >= resend_interval:
                last_send = now
                self.goto(lat, lon, alt_m)

            remaining = deadline - time.monotonic()
            msg = self.recv_match("GLOBAL_POSITION_INT", max(min(remaining, resend_interval), 0))
            if msg is None:
                continue

            dist = horizontal_distance_m(msg.lat / 1e7, msg.lon / 1e7, lat, lon)
            now = time.monotonic()
            if now - last_print >= 1.0:
                last_print = now
                print(f"  distance to next corner: {dist:.1f}m")
            if dist <= reach_radius_m:
                return
        raise DroneError(f"fly_to timed out before reaching target within {reach_radius_m}m")

    def rtl_and_wait(self, timeout: float = 120.0) -> None:
        """Command RTL and block until the drone has landed and disarmed."""
        self.set_mode("RTL")

        deadline = time.monotonic() + timeout
        last_print = 0.0
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            msg = self.recv_match(["HEARTBEAT", "GLOBAL_POSITION_INT"], max(remaining, 0))
            if msg is None:
                break
            if msg.get_type() == "GLOBAL_POSITION_INT":
                now = time.monotonic()
                if now - last_print >= 1.0:
                    last_print = now
                    print(f"  alt={msg.relative_alt / 1000.0:.1f}m")
            elif msg.get_type() == "HEARTBEAT":
                if not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                    return
        raise DroneError("RTL did not result in landing/disarm within timeout")

    def set_param(self, name: str, value: float, timeout: float = 10.0) -> None:
        """Set a parameter (vehicle or simulator) and confirm via PARAM_VALUE.

        Used to inject the SIM_BATT_VOLTAGE failure for Part 3 -- the same
        MAVLink parameter protocol used for any real vehicle parameter.
        """
        try:
            self.conn.mav.param_set_send(
                self.conn.target_system,
                self.conn.target_component,
                name.encode("ascii"),
                float(value),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            )
        except OSError as exc:
            raise DroneError(f"lost connection to drone: {exc}") from exc

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            msg = self.recv_match("PARAM_VALUE", max(remaining, 0))
            if msg is None:
                break
            param_id = msg.param_id
            if isinstance(param_id, bytes):
                param_id = param_id.decode("ascii", errors="replace")
            if param_id.rstrip("\x00") != name:
                continue
            if abs(msg.param_value - value) < 1e-3:
                return
            raise DroneError(
                f"param {name} confirmed as {msg.param_value}, expected {value}"
            )
        raise DroneError(f"timed out confirming param '{name}' was set")
