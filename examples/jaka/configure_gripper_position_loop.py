"""Read or safely configure J4340-2EC position-loop KP/KI through USB2CAN.

The script never enables the motor and never sends a motion command.  Parameter
writes require the motor to be disabled.  Use ``--save`` only when the values
have been tested; it writes the complete parameter set to the driver's flash.
"""

import argparse
import math
import struct
import time

from lerobot.robots.jaka_zu7_follower.damiao_usb2can import DamiaoUSB2CAN


def _wait_reply(
    bus: DamiaoUSB2CAN,
    feedback_id: int,
    predicate,
    description: str,
):
    deadline = time.monotonic() + 0.5
    while time.monotonic() < deadline:
        frame = bus.recv(max(0.001, deadline - time.monotonic()))
        if frame is not None and frame.arbitration_id == feedback_id and predicate(frame.data):
            return frame.data
    raise TimeoutError(f"No reply for {description}; check USB2CAN, CAN bitrate and IDs")


def read_register(bus: DamiaoUSB2CAN, motor_id: int, feedback_id: int, register: int, fmt: str):
    bus.send(0x7FF, bytes((motor_id, 0, 0x33, register, 0, 0, 0, 0)))
    data = _wait_reply(
        bus,
        feedback_id,
        lambda payload: payload[:4] == bytes((motor_id, 0, 0x33, register)),
        f"register 0x{register:02X} read",
    )
    return struct.unpack(fmt, data[4:8])[0]


def read_status(bus: DamiaoUSB2CAN, motor_id: int, feedback_id: int) -> int:
    bus.send(0x7FF, bytes((motor_id, 0, 0xCC, 0, 0, 0, 0, 0)))
    data = _wait_reply(
        bus,
        feedback_id,
        lambda payload: len(payload) == 8 and (payload[0] & 0x0F) == motor_id,
        "motor status",
    )
    return data[0] >> 4


def write_register(
    bus: DamiaoUSB2CAN, motor_id: int, feedback_id: int, register: int, value: float
) -> None:
    payload = bytes((motor_id, 0, 0x55, register)) + struct.pack("<f", value)
    bus.send(0x7FF, payload)
    _wait_reply(
        bus,
        feedback_id,
        lambda data: data[:4] == bytes((motor_id, 0, 0x55, register))
        and data[4:8] == payload[4:8],
        f"register 0x{register:02X} write",
    )


def save_parameters(bus: DamiaoUSB2CAN, motor_id: int, feedback_id: int) -> None:
    payload = bytes((motor_id, 0, 0xAA, 1, 0, 0, 0, 0))
    bus.send(0x7FF, payload)
    _wait_reply(
        bus,
        feedback_id,
        lambda data: data[:4] == payload[:4],
        "parameter save",
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--can-id", type=lambda value: int(value, 0), default=0x7)
    parser.add_argument("--feedback-id", type=lambda value: int(value, 0), default=0x14)
    parser.add_argument("--kp-apr", type=float, default=80.0, help="目标位置环 KP；预览默认 80")
    parser.add_argument("--ki-apr", type=float, default=1.0, help="目标位置环 KI；预览默认 1")
    parser.add_argument("--write", action="store_true", help="实际写入 KP_APR/KI_APR")
    parser.add_argument("--save", action="store_true", help="写入 flash；必须同时指定 --write")
    args = parser.parse_args()

    if not 1 <= args.can_id <= 15 or not 0 <= args.feedback_id <= 0x7FF:
        parser.error("CAN ID must be 1-15 and feedback ID must be an 11-bit ID")
    if not all(math.isfinite(value) and value >= 0 for value in (args.kp_apr, args.ki_apr)):
        parser.error("KP_APR and KI_APR must be finite and non-negative")
    if args.save and not args.write:
        parser.error("--save requires --write")

    bus = DamiaoUSB2CAN(args.port)
    try:
        state = read_status(bus, args.can_id, args.feedback_id)
        state_name = {0: "disabled", 1: "enabled", 0xD: "communication lost"}.get(
            state, f"fault 0x{state:X}"
        )
        kp_before = read_register(bus, args.can_id, args.feedback_id, 27, "<f")
        ki_before = read_register(bus, args.can_id, args.feedback_id, 28, "<f")
        print(f"Motor state: 0x{state:X} ({state_name})")
        print(f"Current KP_APR: {kp_before:.6g}")
        print(f"Current KI_APR: {ki_before:.6g}")
        print(f"Requested KP_APR: {args.kp_apr:.6g}")
        print(f"Requested KI_APR: {args.ki_apr:.6g}")

        if not args.write:
            print("Preview only. No parameter write, enable, save, or motion command was sent.")
            return
        if state != 0:
            raise RuntimeError(
                f"Motor must be disabled before parameter write; current state is 0x{state:X} ({state_name})"
            )

        write_register(bus, args.can_id, args.feedback_id, 27, args.kp_apr)
        write_register(bus, args.can_id, args.feedback_id, 28, args.ki_apr)
        if args.save:
            save_parameters(bus, args.can_id, args.feedback_id)
            print("Parameters saved to flash.")

        kp_after = read_register(bus, args.can_id, args.feedback_id, 27, "<f")
        ki_after = read_register(bus, args.can_id, args.feedback_id, 28, "<f")
        print(f"Verified KP_APR: {kp_after:.6g}")
        print(f"Verified KI_APR: {ki_after:.6g}")
        print("No enable or motion command was sent.")
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
