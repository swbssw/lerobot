"""Preview or execute an independent Damiao gripper reset; does not connect to JAKA."""

import argparse
import logging
import math
import time

from lerobot.robots.jaka_zu7_follower.config_jaka_zu7_follower import JAKAZu7FollowerConfig
from lerobot.robots.jaka_zu7_follower.damiao_gripper import DamiaoGripper
from lerobot.robots.jaka_zu7_follower.damiao_usb2can import DamiaoUSB2CAN

if __package__:
    from .probe_gripper import read_register, read_status
else:
    from probe_gripper import read_register, read_status


def reset_gripper(
    config: JAKAZu7FollowerConfig,
    target_deg: float = 0.0,
    *,
    execute: bool = False,
    timeout_s: float = 15.0,
    tolerance_deg: float = 0.2,
) -> None:
    """Preview without enabling; execution streams until measured arrival, then disables."""
    if config.gripper_can_interface != "dm_usb2can" or config.gripper_can_channel is None:
        raise ValueError("This reset script requires the legacy dm_usb2can adapter")
    assert config.gripper_open_deg is not None and config.gripper_closed_deg is not None
    low, high = sorted((config.gripper_open_deg, config.gripper_closed_deg))
    if not math.isfinite(target_deg) or not low <= target_deg <= high:
        raise ValueError("Reset target must be inside the configured jaw limits")
    if any(not math.isfinite(value) or value <= 0 for value in (timeout_s, tolerance_deg)):
        raise ValueError("Timeout and tolerance must be finite and positive")

    if not execute:
        assert config.gripper_can_id is not None and config.gripper_feedback_id is not None
        bus = DamiaoUSB2CAN(config.gripper_can_channel)
        try:
            motor_id, feedback_id = config.gripper_can_id, config.gripper_feedback_id
            mode = read_register(bus, motor_id, feedback_id, 10, "<I")
            position = math.degrees(read_register(bus, motor_id, feedback_id, 0x50, "<f"))
            state = read_status(bus, motor_id, feedback_id)
            print(f"Current: {position:.3f} deg; target: {target_deg:.3f} deg; mode: {mode}; state: {state}")
            print("Preview only. No enable or motion command sent. Add --execute to move the gripper.")
        finally:
            bus.shutdown()
        return

    gripper = DamiaoGripper(config)
    try:
        gripper.connect()
        deadline = time.monotonic() + timeout_s
        settled_samples = 0
        while time.monotonic() < deadline:
            tick = time.monotonic()
            gripper.move_to_deg(target_deg)
            position = gripper.read_position_deg()
            settled_samples = settled_samples + 1 if abs(position - target_deg) <= tolerance_deg else 0
            if settled_samples >= 3:
                print(f"Gripper reset complete: measured {position:.3f} deg. Disabling gripper.")
                return
            # Keep commands flowing within the motor communication timeout.
            time.sleep(max(0.0, 0.02 - (time.monotonic() - tick)))
        raise TimeoutError("Gripper did not reach the target; it may be obstructed or current-limited")
    finally:
        gripper.disconnect()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--can-id", type=lambda value: int(value, 0), default=0x7)
    parser.add_argument("--feedback-id", type=lambda value: int(value, 0), default=0x14)
    parser.add_argument("--open-deg", type=float, required=True)
    parser.add_argument("--closed-deg", type=float, default=120.0)
    parser.add_argument("--target-deg", type=float, default=0.0)
    parser.add_argument("--speed-deg-s", type=float, default=90.0)
    parser.add_argument("--control-mode", type=int, choices=(2, 4), default=2)
    parser.add_argument("--current-limit-pct", type=float, default=None)
    parser.add_argument("--timeout-s", type=float, default=15.0)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    if args.control_mode == 4 and args.current_limit_pct is None:
        args.current_limit_pct = 1.61
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s", force=True)
    config = JAKAZu7FollowerConfig(
        ip_address="unused",  # No JAKA SDK connection is created.
        gripper_can_channel=args.port,
        gripper_can_interface="dm_usb2can",
        gripper_can_id=args.can_id,
        gripper_feedback_id=args.feedback_id,
        gripper_open_deg=args.open_deg,
        gripper_closed_deg=args.closed_deg,
        gripper_max_velocity_deg_s=args.speed_deg_s,
        gripper_control_mode=args.control_mode,
        gripper_current_limit_pct=args.current_limit_pct,
    )
    try:
        reset_gripper(config, args.target_deg, execute=args.execute, timeout_s=args.timeout_s)
    except KeyboardInterrupt:
        raise SystemExit("Gripper reset interrupted; disable requested.") from None


if __name__ == "__main__":
    main()
