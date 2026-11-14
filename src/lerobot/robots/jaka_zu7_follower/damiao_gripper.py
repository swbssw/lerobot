"""Opt-in J4340-2EC gripper with position-velocity or current-limited position control."""

import importlib
import logging
import math
import struct
import time
from contextlib import suppress
from typing import Any

from .config_jaka_zu7_follower import JAKAZu7FollowerConfig
from .damiao_usb2can import DamiaoUSB2CAN

logger = logging.getLogger(__name__)


class DamiaoGripper:
    def __init__(self, config: JAKAZu7FollowerConfig):
        self.config = config
        self.bus: Any | None = None
        self._can: Any | None = None
        self._enabled = False
        self._start_leader_pct: float | None = None
        self._start_motor_deg: float | None = None
        self._command_deg: float | None = None
        self._last_report_time = 0.0

    @property
    def command_id(self) -> int:
        assert self.config.gripper_can_id is not None
        offset = 0x300 if self.config.gripper_control_mode == 4 else 0x100
        return offset + self.config.gripper_can_id

    def connect(self) -> None:
        try:
            if self.config.gripper_can_interface == "dm_usb2can":
                assert self.config.gripper_can_channel is not None
                self.bus = DamiaoUSB2CAN(self.config.gripper_can_channel)
            else:
                try:
                    self._can = importlib.import_module("can")
                except ImportError as exc:
                    raise ImportError(
                        "J4340-2EC requires the python-can dependency (lerobot[damiao])"
                    ) from exc
                self.bus = self._can.interface.Bus(
                    channel=self.config.gripper_can_channel,
                    interface=self.config.gripper_can_interface,
                    bitrate=1_000_000,
                )
            for register, expected in (
                (8, self.config.gripper_can_id),
                (7, self.config.gripper_feedback_id),
                (10, self.config.gripper_control_mode),
            ):
                actual = self._read_parameter(register, "<I")
                if actual != expected:
                    raise RuntimeError(
                        f"J4340-2EC register {register} is {actual}, expected {expected}; "
                        "check CAN IDs and select the configured mode in the Damiao tool "
                        "(2=position-velocity mode, 4=force-position mode)"
                    )
            timeout_ticks = self._read_parameter(9, "<I")
            if not 0 < timeout_ticks <= 40_000:
                raise RuntimeError(
                    "J4340-2EC CAN communication timeout must be set to 1-40000 ticks "
                    "(50 us each, at most 2 s) in the Damiao tool"
                )
            initial_deg = math.degrees(self._read_parameter(0x50, "<f"))
            self._check_angle(initial_deg)
            self._start_motor_deg = initial_deg
            self._command_deg = initial_deg
            logger.info(
                "Gripper mode=%d, speed cap=%.2f deg/s, phase-current cap=%s%% (not jaw force in N)",
                self.config.gripper_control_mode,
                self.config.gripper_max_velocity_deg_s,
                self.config.gripper_current_limit_pct,
            )
            logger.info(
                "Gripper limits: open=%.3f deg, closed=%.3f deg; startup hold=%.3f deg",
                self.config.gripper_open_deg,
                self.config.gripper_closed_deg,
                initial_deg,
            )
            # Set the target to the measured position while disabled, before enabling torque.
            self._send_position(initial_deg)
            state = self._read_status(allowed_states=(0, 1, 0xD))
            if state == 0xD:
                # Only a fresh connection may recover a previous communication-loss fault.
                # Official J4340 manual p17: FF*7 FB clears faults; FE would change zero.
                logger.warning("J4340 startup communication-loss fault D: clearing once before enable")
                self._send_frame(self.command_id, bytes.fromhex("ff ff ff ff ff ff ff fb"))
                self._read_status(allowed_states=(0,))
                initial_deg = math.degrees(self._read_parameter(0x50, "<f"))
                self._check_angle(initial_deg)
                self._start_motor_deg = initial_deg
                self._command_deg = initial_deg
                self._send_position(initial_deg)
                self._read_status(allowed_states=(0,))
            # Attempt disable on cleanup even if enable transmission is interrupted.
            self._enabled = True
            self._send_frame(self.command_id, bytes.fromhex("ff ff ff ff ff ff ff fc"))
            self._read_status()
            self._send_position(initial_deg)
            self._read_status()
            logger.info(
                "J4340 enabled: feedback state=1, CAN ID=0x%X, feedback ID=0x%X, hold=%.3f deg",
                self.config.gripper_can_id,
                self.config.gripper_feedback_id,
                initial_deg,
            )
        except BaseException:
            with suppress(Exception):
                self.disconnect()
            raise

    def _send_frame(self, arbitration_id: int, data: bytes) -> None:
        if self.bus is None:
            raise RuntimeError("J4340-2EC CAN bus is not connected")
        if isinstance(self.bus, DamiaoUSB2CAN):
            self.bus.send(arbitration_id, data)
        else:
            self.bus.send(self._can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False))

    def _receive(self, predicate: Any, timeout_s: float = 0.05) -> Any:
        if self.bus is None:
            raise RuntimeError("J4340-2EC CAN bus is not connected")
        deadline = time.monotonic() + timeout_s
        while (remaining := deadline - time.monotonic()) > 0:
            message = self.bus.recv(timeout=remaining)
            if message is not None and predicate(message):
                return message
        raise TimeoutError("J4340-2EC feedback timed out; check power, CAN wiring, bitrate and IDs")

    def _read_parameter(self, register: int, fmt: str) -> int | float:
        assert self.config.gripper_can_id is not None
        assert self.config.gripper_feedback_id is not None
        motor_id = self.config.gripper_can_id
        self._send_frame(0x7FF, bytes((motor_id, 0, 0x33, register, 0, 0, 0, 0)))
        message = self._receive(
            lambda msg: (
                msg.arbitration_id == self.config.gripper_feedback_id
                and len(msg.data) == 8
                and msg.data[0] == motor_id
                and msg.data[1] == 0
                and msg.data[2] == 0x33
                and msg.data[3] == register
            )
        )
        value = struct.unpack(fmt, bytes(message.data[4:8]))[0]
        if isinstance(value, float) and not math.isfinite(value):
            raise RuntimeError(f"J4340-2EC register {register} returned a non-finite value")
        return value

    def _read_status(self, allowed_states: tuple[int, ...] = (1,)) -> int:
        assert self.config.gripper_feedback_id is not None
        assert self.config.gripper_can_id is not None
        message = self._receive(
            lambda msg: (
                msg.arbitration_id == self.config.gripper_feedback_id
                and len(msg.data) == 8
                and (msg.data[0] & 0x0F) == self.config.gripper_can_id
            ),
        )
        state = message.data[0] >> 4
        if state not in allowed_states:
            if state == 0:
                raise RuntimeError("J4340-2EC disabled: motor feedback state=0; stopping teleoperation")
            if state == 0xD:
                raise RuntimeError(
                    "J4340-2EC fault status D: communication lost. Stop other controller programs "
                    "and check CAN/USB and power. No running-session recovery or repeated clear is attempted."
                )
            raise RuntimeError(f"J4340-2EC fault status {state:X}")
        return state

    def _check_angle(self, angle_deg: float) -> None:
        assert self.config.gripper_open_deg is not None
        assert self.config.gripper_closed_deg is not None
        low, high = sorted((self.config.gripper_open_deg, self.config.gripper_closed_deg))
        if not math.isfinite(angle_deg) or not low <= angle_deg <= high:
            raise RuntimeError(
                f"J4340-2EC position {angle_deg:.2f} deg is outside configured jaw limits [{low}, {high}]"
            )

    def _send_position(self, angle_deg: float) -> None:
        speed_rad_s = math.radians(self.config.gripper_max_velocity_deg_s)
        if self.config.gripper_control_mode == 4:
            assert self.config.gripper_current_limit_pct is not None
            # Manual: little-endian float angle, uint16 velocity (rad/s * 100),
            # uint16 phase-current fraction * 10000. Round down to respect caps.
            payload = struct.pack(
                "<fHH", math.radians(angle_deg), int(speed_rad_s * 100),
                int(self.config.gripper_current_limit_pct * 100),
            )
        else:
            payload = struct.pack("<ff", math.radians(angle_deg), speed_rad_s)
        self._send_frame(self.command_id, payload)

    def move_to_deg(self, angle_deg: float) -> None:
        """Command an absolute jaw angle, respecting configured travel/current/speed limits."""
        if not self._enabled:
            raise RuntimeError("J4340-2EC gripper is not enabled")
        self._check_angle(angle_deg)
        self._send_position(angle_deg)
        self._read_status()
        self._command_deg = angle_deg

    def read_position_deg(self) -> float:
        if not self._enabled:
            raise RuntimeError("J4340-2EC gripper is not enabled")
        angle = math.degrees(self._read_parameter(0x50, "<f"))
        self._check_angle(angle)
        return angle

    def send(self, leader_pct: float) -> float:
        if not math.isfinite(leader_pct) or not 0 <= leader_pct <= 100:
            raise ValueError("SO-101 gripper action must be between 0 and 100 percent")
        if not self._enabled or self._start_motor_deg is None:
            raise RuntimeError("J4340-2EC gripper is not enabled")
        if self._start_leader_pct is None:
            self._start_leader_pct = leader_pct
        assert self.config.gripper_open_deg is not None
        assert self.config.gripper_closed_deg is not None
        target = (
            self._start_motor_deg
            + (leader_pct - self._start_leader_pct)
            * self.config.gripper_leader_sign
            * (self.config.gripper_closed_deg - self.config.gripper_open_deg)
            / 100
        )
        low, high = sorted((self.config.gripper_open_deg, self.config.gripper_closed_deg))
        unclamped_target = target
        target = max(low, min(high, target))
        self._send_position(target)
        self._read_status()
        self._command_deg = target
        now = time.monotonic()
        if now - self._last_report_time >= 1.0:
            logger.info(
                "Gripper: leader=%.2f%%, start=%.2f%%, command=%.3f deg (not measured), "
                "clamped=%s, leader_sign=%+d, motor feedback state=1",
                leader_pct,
                self._start_leader_pct,
                target,
                target != unclamped_target,
                self.config.gripper_leader_sign,
            )
            self._last_report_time = now
        return self._to_percent(target)

    def read(self) -> float:
        return self._to_percent(self.read_position_deg())

    def _to_percent(self, angle_deg: float) -> float:
        assert self.config.gripper_open_deg is not None
        assert self.config.gripper_closed_deg is not None
        return (
            100
            * (angle_deg - self.config.gripper_open_deg)
            / (self.config.gripper_closed_deg - self.config.gripper_open_deg)
        )

    def disconnect(self) -> None:
        error: Exception | None = None
        if self.bus is not None:
            try:
                if self._enabled:
                    self._send_frame(self.command_id, bytes.fromhex("ff ff ff ff ff ff ff fd"))
            except Exception as exc:
                error = exc
            try:
                self.bus.shutdown()
            except Exception as exc:
                error = error or exc
        self.bus = None
        self._enabled = False
        self._start_leader_pct = None
        self._start_motor_deg = None
        self._command_deg = None
        if error is not None:
            raise RuntimeError("Failed to disconnect J4340-2EC gripper") from error
