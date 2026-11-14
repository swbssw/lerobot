"""Diagnose JAKA servo hold or joint arrival after a simultaneous step command.

The default streams the current pose without leader input. Nonzero step commands
all six joints to jump by the same amount in one servo tick. The JAKA SDK warns
that servo_j requires planned trajectories; direct steps can cause violent shaking.
They require explicit --allow-direct-step acknowledgement.

Safety: the robot must already be powered and enabled.
Keep the emergency stop accessible; Ctrl+C stops the script and exits servo mode.
"""

import argparse
import math
import time
from typing import Any

from lerobot.robots.jaka_zu7_follower.sdk_bridge import JakaSDKBridge

if __package__:
    from . import _system
else:
    import _system

JAKA_ZU7_JOINTS = (
    "shoulder_pan",
    "heavy_joint",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)
SERVO_PERIOD_S = 0.008
ARRIVAL_TOL_DEG = 0.5


def check_result(result: Any, operation: str) -> Any:
    if not isinstance(result, (list, tuple)) or not result or result[0] != 0:
        raise RuntimeError(f"JAKA {operation} failed: {result}")
    return result


def read_joints_rad(sdk: JakaSDKBridge) -> list[float]:
    result = check_result(sdk.get_joint_position(), "get_joint_position")
    joints = list(result[1])
    if len(joints) != 6 or not all(math.isfinite(value) for value in joints):
        raise RuntimeError(f"Invalid JAKA joint feedback: {joints}")
    return joints


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip", required=True)
    parser.add_argument("--sdk-python", default=None, help="SDK 解释器(默认按所选系统填入)")
    parser.add_argument("--sdk-dir", default=None, help="jkrc SDK 目录(默认按所选系统填入)")
    parser.add_argument("--step-deg", type=float, default=0.0, help="六个关节同时迈的步长(度);默认 0 为原位保持")
    parser.add_argument("--allow-direct-step", action="store_true",
                        help="明确允许未规划的六关节瞬时步进;可能导致剧烈抖动")
    parser.add_argument("--hold-s", type=float, default=4.0, help="迈步后保持目标的时间(秒)")
    parser.add_argument("--readback-interval-s", type=float, default=0.2,
                        help="运行中读取实际角度的间隔(秒);0 关闭,避免打断伺服发送")
    parser.add_argument("--nlf", action=argparse.BooleanOptionalAction, default=True,
                        help="开关节 NLF 滤波(默认开,和遥操一致)")
    parser.add_argument("--nlf-vel", type=float, default=60.0, help="NLF 速度上限(度/秒)")
    parser.add_argument("--nlf-acc", type=float, default=300.0, help="NLF 加速度上限(度/秒^2)")
    parser.add_argument("--nlf-jerk", type=float, default=1000.0, help="NLF 加加速度上限(度/秒^3)")
    args = parser.parse_args()
    defaults = _system.choose()
    args.sdk_python = args.sdk_python or defaults["sdk_python"]
    args.sdk_dir = args.sdk_dir or defaults["sdk_dir"]

    if not math.isfinite(args.step_deg) or args.step_deg < 0:
        parser.error("step-deg 必须为非负数")
    if args.step_deg > 0 and not args.allow_direct_step:
        parser.error("非零 step-deg 是未规划的瞬时六关节步进,可能剧烈抖动;本诊断默认只允许原位保持")
    if not math.isfinite(args.hold_s) or args.hold_s <= 0:
        parser.error("hold-s 必须为正数")
    if not math.isfinite(args.readback_interval_s) or args.readback_interval_s < 0:
        parser.error("readback-interval-s 必须为非负数")

    step_rad = math.radians(args.step_deg)
    if args.step_deg == 0:
        print(f"原位保持测试:保持启动时的六关节角度 {args.hold_s} 秒,不发送位移目标")
    else:
        print(f"步进测试:6 个关节同时迈 {args.step_deg} 度,然后保持 {args.hold_s} 秒")
    if args.readback_interval_s > 0:
        print(f"判据:某关节到位 = 距离目标 < {ARRIVAL_TOL_DEG} 度")
    print("注意:保持急停可及,感觉异样立刻 Ctrl+C。")

    try:
        with JakaSDKBridge(args.ip, args.sdk_python, args.sdk_dir, timeout_s=15.0) as sdk:
            check_result(sdk.login(), "login")
            state = check_result(sdk.get_robot_state(), "get_robot_state")[1]
            if state[0] or not state[1] or not state[2]:
                raise SystemExit(
                    f"机械臂未就绪(急停={state[0]}, 上电={state[1]}, 使能={state[2]})。"
                    "请先上电并使能后再运行本脚本。"
                )

            base_rad = read_joints_rad(sdk)
            print("起始 J1-J6(度):", [round(math.degrees(v), 3) for v in base_rad])
            target_rad = [b + step_rad for b in base_rad]
            print("目标 J1-J6(度):", [round(math.degrees(v), 3) for v in target_rad])

            if args.nlf:
                check_result(
                    sdk.servo_move_use_joint_NLF(args.nlf_vel, args.nlf_acc, args.nlf_jerk),
                    "servo_move_use_joint_NLF",
                )
            else:
                check_result(sdk.servo_move_use_none_filter(), "servo_move_use_none_filter")
            check_result(sdk.servo_move_enable(True), "servo_move_enable(True)")

            start = time.monotonic()
            next_sample = start
            arrived: list[float | None] = [None] * 6
            if args.readback_interval_s > 0:
                print("  t(s)   误差J1  误差J2  误差J3  误差J4  误差J5  误差J6 (度,越接近 0 表示已到位)")
            sent_count = 0
            try:
                while True:
                    now = time.monotonic()
                    t = now - start
                    if t >= args.hold_s:
                        break
                    check_result(sdk.servo_j(target_rad, 0), "servo_j")
                    sent_count += 1

                    if args.readback_interval_s > 0 and now >= next_sample:
                        actual = read_joints_rad(sdk)
                        errors = [math.degrees(actual[j] - target_rad[j]) for j in range(6)]
                        for j in range(6):
                            if arrived[j] is None and abs(errors[j]) < ARRIVAL_TOL_DEG:
                                arrived[j] = t
                        print(f"{t:6.2f}  " + "  ".join(f"{e:+6.2f}" for e in errors))
                        while next_sample <= now:
                            next_sample += args.readback_interval_s

                    remaining = SERVO_PERIOD_S - (time.monotonic() - now)
                    if remaining > 0:
                        time.sleep(remaining)
            finally:
                if not sdk.closed:
                    result = sdk.servo_move_enable(False)
                    if (not isinstance(result, (list, tuple)) or not result) or result[0] not in (0, -61):
                        raise RuntimeError(f"JAKA servo_move_enable(False) failed: {result}")

            print(f"发送频率: {sent_count / (time.monotonic() - start):.1f} Hz")
            if args.readback_interval_s > 0:
                print("\n各关节到位耗时(秒,None = 保持期内未到位):")
                for j, name in enumerate(JAKA_ZU7_JOINTS):
                    t_arr = arrived[j]
                    if t_arr is None:
                        print(f"  J{j + 1} {name:<14} 未到位")
                    else:
                        print(f"  J{j + 1} {name:<14} {t_arr:5.2f} 秒")
    except KeyboardInterrupt:
        print("已中断;伺服模式已退出。确认机械臂已停止。")
        raise SystemExit(130) from None


if __name__ == "__main__":
    main()
