"""Preview or execute the JAKA reference move followed by a gripper opening."""

import argparse
import logging
import math
import time
from typing import Any

from lerobot.robots.jaka_zu7_follower.config_jaka_zu7_follower import JAKAZu7FollowerConfig
from lerobot.robots.jaka_zu7_follower.sdk_bridge import JakaSDKBridge

if __package__:
    from . import _system
    from .reset_gripper import reset_gripper
else:
    import _system
    from reset_gripper import reset_gripper

REFERENCE_DEG = [0.0, 90.0, 0.0, 90.0, 180.0, 0.0]


def check_result(result: Any, operation: str) -> Any:
    if not isinstance(result, (list, tuple)) or not result or result[0] != 0:
        raise RuntimeError(f"JAKA {operation} failed: {result}")
    return result


def read_joints_deg(sdk: JakaSDKBridge) -> list[float]:
    result = check_result(sdk.get_joint_position(), "get_joint_position")
    joints = [math.degrees(value) for value in result[1]]
    if len(joints) != 6 or not all(math.isfinite(value) for value in joints):
        raise RuntimeError(f"Invalid JAKA joint feedback: {joints}")
    return joints


def reset_to_reference(
    sdk: JakaSDKBridge,
    target_deg: list[float],
    *,
    execute: bool = False,
    speed_deg_s: float = 20.0,
    acceleration_deg_s2: float = 40.0,
    timeout_s: float = 120.0,
    tolerance_deg: float = 0.5,
    gripper_config: JAKAZu7FollowerConfig | None = None,
    gripper_timeout_s: float = 15.0,
) -> None:
    if len(target_deg) != 6 or not all(math.isfinite(value) for value in target_deg):
        raise ValueError("The target must contain six finite angles in degrees")
    limits = (speed_deg_s, acceleration_deg_s2, timeout_s, tolerance_deg, gripper_timeout_s)
    if any(not math.isfinite(value) or value <= 0 for value in limits):
        raise ValueError("Speed, acceleration, timeout and tolerance must be finite and positive")

    state = check_result(sdk.get_robot_state(), "get_robot_state")[1]
    joints = read_joints_deg(sdk)
    print("Current J1-J6 (degrees):", [round(value, 3) for value in joints])
    print("Target  J1-J6 (degrees):", target_deg)
    print(f"Speed: {speed_deg_s} degrees/s; acceleration: {acceleration_deg_s2} degrees/s^2")
    print(f"Emergency stop: {state[0]}; powered: {state[1]}; enabled: {state[2]}")
    if gripper_config is not None:
        assert gripper_config.gripper_open_deg is not None
        print(
            f"Gripper reset: open to {gripper_config.gripper_open_deg} deg AFTER JAKA motion; "
            f"speed={gripper_config.gripper_max_velocity_deg_s} deg/s; "
            f"mode={gripper_config.gripper_control_mode}; "
            f"current cap={gripper_config.gripper_current_limit_pct}% (not calibrated jaw force). "
            "Opening releases any held object."
        )
    if not execute:
        if gripper_config is not None:
            reset_gripper(gripper_config, gripper_config.gripper_open_deg, execute=False)
        print("Preview only. No motion sent. Stop teleoperation and inspect the path before using --execute.")
        return

    if state[0] or not state[1] or not state[2]:
        raise RuntimeError("Reset requires emergency stop released and the robot powered and enabled")
    if not check_result(sdk.is_in_pos(), "is_in_pos")[1]:
        raise RuntimeError("Robot is moving. Stop teleoperation and wait for it to stop before resetting")
    arm_at_reference = (
        max(abs(current - target) for current, target in zip(joints, target_deg, strict=True))
        <= tolerance_deg
    )
    if arm_at_reference and gripper_config is None:
        print("Already at the reference pose.")
        return

    in_servo = check_result(sdk.is_in_servomove(), "is_in_servomove")[1]
    print(f"Streaming servo mode: {in_servo}")
    if in_servo:
        check_result(sdk.servo_move_enable(False), "servo_move_enable(False)")
        if check_result(sdk.is_in_servomove(), "is_in_servomove")[1]:
            raise RuntimeError("Robot is still in streaming servo mode; no reset motion sent")
    else:
        print("Already outside streaming servo mode; skipping servo_move_enable(False).")
    # Mode queries temporarily release and restore the SDK command connection.
    # Recheck state after reconnecting before submitting any movement.
    state = check_result(sdk.get_robot_state(), "get_robot_state")[1]
    if state[0] or not state[1] or not state[2]:
        raise RuntimeError(f"Robot is not ready after the servo-mode query: {state}")
    if not check_result(sdk.is_in_pos(), "is_in_pos")[1]:
        raise RuntimeError("Robot started moving during reset preparation; no reset motion sent")
    check_result(sdk.get_tool_id(), "get_tool_id")
    # JAKA is reset first so the arm reaches its safe reference pose before the
    # gripper opens.  The gripper reset is performed after this motion below.
    try:
        check_result(
            sdk.joint_move_extend(
                [math.radians(value) for value in target_deg],
                0,  # Absolute joint angles.
                False,  # Poll rather than blocking the SDK pipe for the whole move.
                math.radians(speed_deg_s),
                math.radians(acceleration_deg_s2),
                0.0,  # Zero endpoint tolerance.
            ),
            "joint_move_extend",
        )
        deadline = time.monotonic() + timeout_s
        while True:
            joints = read_joints_deg(sdk)
            stopped = check_result(sdk.is_in_pos(), "is_in_pos")[1]
            error = max(abs(current - target) for current, target in zip(joints, target_deg, strict=True))
            if stopped and error <= tolerance_deg:
                print("Reset complete. J1-J6 (degrees):", [round(value, 3) for value in joints])
                break
            state = check_result(sdk.get_robot_state(), "get_robot_state")[1]
            if state[0] or not state[1] or not state[2]:
                raise RuntimeError(f"Robot state changed during reset: {state}")
            if time.monotonic() >= deadline:
                raise TimeoutError(f"Reset did not reach the target within {timeout_s}s")
            time.sleep(0.1)
    finally:
        # This stops the nonblocking move on timeout, SDK errors and Ctrl+C as
        # well as after completion. A closed bridge already invokes worker EOF cleanup.
        if not sdk.closed:
            check_result(sdk.motion_abort(), "motion_abort")

    if gripper_config is not None:
        assert gripper_config.gripper_open_deg is not None
        # reset_gripper verifies measured arrival and disables on all exit paths.
        reset_gripper(
            gripper_config,
            gripper_config.gripper_open_deg,
            execute=True,
            timeout_s=gripper_timeout_s,
        )
        print("JAKA reset complete; gripper reset complete.")


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip", required=True)
    parser.add_argument("--sdk-python", default=None, help="SDK 解释器(默认按所选系统填入)")
    parser.add_argument("--sdk-dir", default=None, help="jkrc SDK 目录(默认按所选系统填入)")
    parser.add_argument("--target-deg", type=float, nargs=6, default=REFERENCE_DEG)
    parser.add_argument("--speed-deg-s", type=float, default=20.0)
    parser.add_argument("--acceleration-deg-s2", type=float, default=40.0)
    parser.add_argument("--timeout-s", type=float, default=120.0)
    parser.add_argument("--skip-gripper", action="store_true", help="Only reset JAKA; skip the gripper")
    parser.add_argument(
        "--gripper-port", default="/dev/serial/by-id/usb-HDSC_CDC_Device_00000000050C-if00"
    )
    parser.add_argument("--gripper-can-id", type=lambda value: int(value, 0), default=0x7)
    parser.add_argument("--gripper-feedback-id", type=lambda value: int(value, 0), default=0x14)
    parser.add_argument("--gripper-open-deg", type=float, default=0.0)
    parser.add_argument("--gripper-closed-deg", type=float, default=120.0)
    parser.add_argument("--gripper-speed-deg-s", type=float, default=90.0)
    parser.add_argument("--gripper-control-mode", type=int, choices=(2, 4), default=4)
    parser.add_argument("--gripper-current-limit-pct", type=float, default=1.61)
    parser.add_argument("--gripper-timeout-s", type=float, default=15.0)
    parser.add_argument("--execute", action="store_true", help="Open gripper then move JAKA; otherwise preview")
    return parser


def make_gripper_config(args: argparse.Namespace) -> JAKAZu7FollowerConfig | None:
    if not args.skip_gripper:
        return JAKAZu7FollowerConfig(
            ip_address=args.ip,
            gripper_can_channel=args.gripper_port,
            gripper_can_interface="dm_usb2can",
            gripper_can_id=args.gripper_can_id,
            gripper_feedback_id=args.gripper_feedback_id,
            gripper_open_deg=args.gripper_open_deg,
            gripper_closed_deg=args.gripper_closed_deg,
            gripper_max_velocity_deg_s=args.gripper_speed_deg_s,
            gripper_control_mode=args.gripper_control_mode,
            gripper_current_limit_pct=(
                args.gripper_current_limit_pct if args.gripper_control_mode == 4 else None
            ),
        )
    return None


def main() -> None:
    args = make_parser().parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s", force=True)
    gripper_config = make_gripper_config(args)
    defaults = _system.choose()
    args.sdk_python = args.sdk_python or defaults["sdk_python"]
    args.sdk_dir = args.sdk_dir or defaults["sdk_dir"]
    try:
        with JakaSDKBridge(args.ip, args.sdk_python, args.sdk_dir, timeout_s=15.0) as sdk:
            check_result(sdk.login(), "login")
            reset_to_reference(
                sdk,
                args.target_deg,
                execute=args.execute,
                speed_deg_s=args.speed_deg_s,
                acceleration_deg_s2=args.acceleration_deg_s2,
                timeout_s=args.timeout_s,
                gripper_config=gripper_config,
                gripper_timeout_s=args.gripper_timeout_s,
            )
    except TimeoutError as exc:
        raise SystemExit(
            f"Reset timed out: {exc}. No automatic retry. A motion command may already have reached "
            "the controller; verify the robot has stopped before retrying."
        ) from exc
    except KeyboardInterrupt:
        print("Reset interrupted; stopping motion was requested. Verify the robot has stopped.")
        raise SystemExit(130) from None


if __name__ == "__main__":
    main()
