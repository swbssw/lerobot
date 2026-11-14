"""Read J4340-2EC settings through a legacy Damiao USB2CAN adapter; never enable or move it."""

import argparse
import math
import struct
import time

from lerobot.robots.jaka_zu7_follower.damiao_usb2can import DamiaoUSB2CAN


def read_register(bus: DamiaoUSB2CAN, motor_id: int, feedback_id: int, register: int, fmt: str):
    bus.send(0x7FF, bytes((motor_id, 0, 0x33, register, 0, 0, 0, 0)))
    deadline = time.monotonic() + 0.3
    while time.monotonic() < deadline:
        frame = bus.recv(max(0.001, deadline - time.monotonic()))
        if frame is None:
            continue
        if frame.arbitration_id == feedback_id and frame.data[:4] == bytes((motor_id, 0, 0x33, register)):
            return struct.unpack(fmt, frame.data[4:8])[0]
    raise TimeoutError(
        f"No reply for register 0x{register:02X}; check USB2CAN mode, power, CAN wiring and IDs"
    )


def read_status(bus: DamiaoUSB2CAN, motor_id: int, feedback_id: int) -> int:
    """Request feedback only; 0xCC is the documented status refresh, not enable."""
    bus.send(0x7FF, bytes((motor_id, 0, 0xCC, 0, 0, 0, 0, 0)))
    deadline = time.monotonic() + 0.3
    while time.monotonic() < deadline:
        frame = bus.recv(max(0.001, deadline - time.monotonic()))
        if (
            frame is not None
            and frame.arbitration_id == feedback_id
            and len(frame.data) == 8
            and frame.data[0] & 0x0F == motor_id
        ):
            return frame.data[0] >> 4
    raise TimeoutError("No motor status reply to read-only refresh")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM1")
    parser.add_argument("--can-id", type=lambda value: int(value, 0), default=0x7)
    parser.add_argument("--feedback-id", type=lambda value: int(value, 0), default=0x14)
    args = parser.parse_args()
    if not 1 <= args.can_id <= 15 or not 0 <= args.feedback_id <= 0x7FF:
        parser.error("CAN ID must be 1-15 and feedback ID must be an 11-bit ID")

    bus = DamiaoUSB2CAN(args.port)
    try:
        for label, register, fmt in (
            ("CAN ID", 8, "<I"),
            ("Master ID", 7, "<I"),
            ("Control mode", 10, "<I"),
            ("CAN timeout ticks", 9, "<I"),
            ("ACC (rotor rad/s^2)", 4, "<f"),
            ("DEC (rotor rad/s^2)", 5, "<f"),
            ("MAX_SPD (rotor rad/s)", 6, "<f"),
            ("PMAX (rad)", 21, "<f"),
            ("VMAX (rad/s)", 22, "<f"),
            ("TMAX (N m)", 23, "<f"),
            ("KP_APR", 27, "<f"),
            ("KI_APR", 28, "<f"),
            ("Imax (A)", 59, "<f"),
            ("VBus (V)", 60, "<f"),
        ):
            value = read_register(bus, args.can_id, args.feedback_id, register, fmt)
            if fmt == "<f":
                print(f"{label}: {value:.6g}")
            else:
                print(f"{label}: {value}")
        angle_deg = math.degrees(read_register(bus, args.can_id, args.feedback_id, 0x50, "<f"))
        print(f"Motor position: {angle_deg:.3f} deg")
        state = read_status(bus, args.can_id, args.feedback_id)
        label = {0: "disabled", 1: "enabled", 0xD: "communication lost"}.get(state, "fault/other")
        print(f"Motor feedback state: 0x{state:X} ({label})")
        print("Read-only probe complete. No enable or motion command was sent.")
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
