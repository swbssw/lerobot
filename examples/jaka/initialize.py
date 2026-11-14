"""Preview or execute JAKA power/enable, arm reset, gripper opening, and hold."""

import logging
import math
import time

from lerobot.robots.jaka_zu7_follower.config_jaka_zu7_follower import JAKAZu7FollowerConfig
from lerobot.robots.jaka_zu7_follower.damiao_gripper import DamiaoGripper
from lerobot.robots.jaka_zu7_follower.sdk_bridge import JakaSDKBridge

if __package__:
    from . import _system
    from .power_on import power_on_and_enable
    from .reset_to_reference import check_result, make_gripper_config, make_parser, reset_to_reference
else:
    import _system
    from power_on import power_on_and_enable
    from reset_to_reference import check_result, make_gripper_config, make_parser, reset_to_reference


def initialize(
    sdk: JakaSDKBridge,
    target_deg: list[float],
    config: JAKAZu7FollowerConfig | None,
    *,
    execute: bool = False,
    power_on_delay_s: float = 8.0,
    speed_deg_s: float = 20.0,
    acceleration_deg_s2: float = 40.0,
    timeout_s: float = 120.0,
    gripper_timeout_s: float = 15.0,
) -> None:
    """Reuse reset checks and cleanup; do not energize anything in preview mode."""
    if len(target_deg) != 6 or not all(math.isfinite(value) for value in target_deg):
        raise ValueError("Target must contain six finite angles")
    if not math.isfinite(power_on_delay_s) or power_on_delay_s < 0:
        raise ValueError("Power-on delay must be finite and non-negative")
    if any(not math.isfinite(value) or value <= 0 for value in (
        speed_deg_s, acceleration_deg_s2, timeout_s, gripper_timeout_s
    )):
        raise ValueError("Speeds and timeouts must be finite and positive")
    if execute:
        power_on_and_enable(sdk, power_on_delay_s)
    else:
        print("Preview: execution will power/enable JAKA, reset JAKA, then open the gripper.")
    reset_to_reference(
        sdk, target_deg, execute=execute, speed_deg_s=speed_deg_s,
        acceleration_deg_s2=acceleration_deg_s2, timeout_s=timeout_s,
        gripper_config=config, gripper_timeout_s=gripper_timeout_s,
    )


def hold_gripper(config: JAKAZu7FollowerConfig) -> None:
    """Own the adapter until Ctrl+C; keep alive at 50 Hz and stop on any fault."""
    gripper = DamiaoGripper(config)
    try:
        gripper.connect()
        target = gripper.read_position_deg()
        print(
            f"初始化完成。4340 保持当前位置 {target:.3f}°（50 Hz 保持指令）。\n"
            "此终端正在占用夹爪串口。开始遥操前，按 Ctrl+C 失能并释放串口，"
            "再运行遥操命令；遥操会重新使能。JAKA 保持上电使能。",
            flush=True,
        )
        while True:
            tick = time.monotonic()
            gripper.move_to_deg(target)
            time.sleep(max(0.0, 0.02 - (time.monotonic() - tick)))
    finally:
        gripper.disconnect()


def main() -> None:
    parser = make_parser()
    parser.description = __doc__
    parser.add_argument("--power-on-delay-s", type=float, default=8.0)
    parser.add_argument(
        "--exit-after-reset", action="store_true",
        help="Exit after reset with gripper disabled; otherwise hold until Ctrl+C",
    )
    args = parser.parse_args()
    config = make_gripper_config(args)
    defaults = _system.choose()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s", force=True)
    print("请停止其他控制程序。4340 需要外部供电；执行会张开夹爪并移动 JAKA。")
    try:
        with JakaSDKBridge(
            args.ip, args.sdk_python or defaults["sdk_python"],
            args.sdk_dir or defaults["sdk_dir"], timeout_s=15.0,
        ) as sdk:
            check_result(sdk.login(), "login")
            initialize(
                sdk, args.target_deg, config, execute=args.execute,
                power_on_delay_s=args.power_on_delay_s, speed_deg_s=args.speed_deg_s,
                acceleration_deg_s2=args.acceleration_deg_s2, timeout_s=args.timeout_s,
                gripper_timeout_s=args.gripper_timeout_s,
            )
        if args.execute and config is not None and not args.exit_after_reset:
            hold_gripper(config)
        elif args.execute:
            print("初始化复位完成；夹爪已失能，JAKA 保持上电使能。")
    except KeyboardInterrupt:
        print("初始化/保持已中断，已请求停止运动并失能夹爪。")
        raise SystemExit(130) from None
    except TimeoutError as exc:
        raise SystemExit(f"初始化/保持超时: {exc}。不自动重试；请确认设备已停止。") from exc


if __name__ == "__main__":
    main()
