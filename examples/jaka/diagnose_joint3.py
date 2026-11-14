"""Diagnose JAKA joint tracking lag — single joint or all six at once.

Single joint (default): commands a slow sine on ONE joint (default 3 = shoulder_lift),
holds the others, and compares command vs actual to measure that joint's lag in isolation.

--all-joints: commands ALL six joints simultaneously with phase-offset sines, so we can
see whether one joint (e.g. joint 3) lags MORE than the others only when everyone moves
together — i.e. reproduce the "joint 3 waits for the others" symptom.

Safety: the robot must already be powered and enabled. Default amplitude is 5 degrees.
Keep the emergency stop accessible; Ctrl+C stops the script and exits servo mode.
"""

import argparse
import math
import time
from typing import Any

from lerobot.robots.jaka_zu7_follower.sdk_bridge import JakaSDKBridge

import _system  # sibling module: interactive OS selection

JAKA_ZU7_JOINTS = (
    "shoulder_pan",
    "heavy_joint",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)
SERVO_PERIOD_S = 0.008


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
    parser.add_argument("--joint", type=int, default=3, choices=range(1, 7),
                        help="单关节模式:要测的关节编号 1-6(默认 3 = shoulder_lift)")
    parser.add_argument("--all-joints", action="store_true",
                        help="全关节模式:6 个关节同时摆动,看谁滞后最多(复现遥操耦合)")
    parser.add_argument("--amplitude-deg", type=float, default=5.0, help="摆动幅度(度)")
    parser.add_argument("--period-s", type=float, default=4.0, help="摆动周期(秒)")
    parser.add_argument("--duration-s", type=float, default=12.0, help="总时长(秒)")
    parser.add_argument("--nlf", action=argparse.BooleanOptionalAction, default=True,
                        help="开关节 NLF 滤波(默认开,和遥操一致;--no-nlf 关掉做对照)")
    parser.add_argument("--nlf-vel", type=float, default=60.0, help="NLF 速度上限(度/秒)")
    parser.add_argument("--nlf-acc", type=float, default=300.0, help="NLF 加速度上限(度/秒^2)")
    parser.add_argument("--nlf-jerk", type=float, default=1000.0, help="NLF 加加速度上限(度/秒^3)")
    args = parser.parse_args()
    defaults = _system.choose()
    args.sdk_python = args.sdk_python or defaults["sdk_python"]
    args.sdk_dir = args.sdk_dir or defaults["sdk_dir"]

    if args.amplitude_deg <= 0 or args.period_s <= 0 or args.duration_s <= 0:
        parser.error("amplitude、period、duration 都必须为正数")

    moving = None if args.all_joints else [args.joint - 1]
    peak_vel = args.amplitude_deg * 2.0 * math.pi / args.period_s
    if args.all_joints:
        print(f"全关节模式:6 个关节同时摆动,幅度 {args.amplitude_deg} 度,周期 {args.period_s} 秒,"
              f"峰值速度约 {peak_vel:.2f} 度/秒")
    else:
        print(f"待测关节:{args.joint}({JAKA_ZU7_JOINTS[args.joint - 1]})")
        print(f"命令正弦:幅度 {args.amplitude_deg} 度,周期 {args.period_s} 秒,峰值速度约 {peak_vel:.2f} 度/秒")
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

            if args.nlf:
                check_result(
                    sdk.servo_move_use_joint_NLF(args.nlf_vel, args.nlf_acc, args.nlf_jerk),
                    "servo_move_use_joint_NLF",
                )
            else:
                check_result(sdk.servo_move_use_none_filter(), "servo_move_use_none_filter")
            check_result(sdk.servo_move_enable(True), "servo_move_enable(True)")

            amp_rad = math.radians(args.amplitude_deg)
            omega = 2.0 * math.pi / args.period_s
            # 全关节模式下给每个关节一个相位偏移,让它们不是完全同步地动
            phase = [i * math.pi / 3.0 for i in range(6)]
            start = time.monotonic()
            next_report = start + 0.5
            if args.all_joints:
                print(f"  t(s)   滞后J1  滞后J2  滞后J3  滞后J4  滞后J5  滞后J6")
            else:
                print(f"  t(s)   命令J{args.joint}(度)  实际J{args.joint}(度)  滞后(度)  其它关节最大漂移(度)")
            try:
                while True:
                    now = time.monotonic()
                    t = now - start
                    if t >= args.duration_s:
                        break
                    target = list(base_rad)
                    if moving is None:
                        for j in range(6):
                            target[j] = base_rad[j] + amp_rad * math.sin(omega * t + phase[j])
                    else:
                        for j in moving:
                            target[j] = base_rad[j] + amp_rad * math.sin(omega * t)
                    check_result(sdk.servo_j(target, 0), "servo_j")

                    if now >= next_report:
                        actual = read_joints_rad(sdk)
                        if args.all_joints:
                            lags = [
                                math.degrees((target[j] - base_rad[j]) - (actual[j] - base_rad[j]))
                                for j in range(6)
                            ]
                            print(f"{t:6.2f}  " + "  ".join(f"{lag:+6.2f}" for lag in lags))
                        else:
                            j = moving[0]
                            cmd_deg = math.degrees(target[j] - base_rad[j])
                            act_deg = math.degrees(actual[j] - base_rad[j])
                            lag_deg = cmd_deg - act_deg
                            held = max(
                                abs(math.degrees(actual[i] - base_rad[i]))
                                for i in range(6)
                                if i != j
                            )
                            print(f"{t:6.2f}  {cmd_deg:+8.2f}  {act_deg:+8.2f}  {lag_deg:+8.2f}  {held:8.2f}")
                        next_report += 0.5

                    remaining = SERVO_PERIOD_S - (time.monotonic() - now)
                    if remaining > 0:
                        time.sleep(remaining)
            finally:
                if not sdk.closed:
                    result = sdk.servo_move_enable(False)
                    # -61(ERR_OPERATION_TIMEOUT)在退出伺服时是误报,伺服实际已退出
                    if (not isinstance(result, (list, tuple)) or not result) or result[0] not in (0, -61):
                        raise RuntimeError(f"JAKA servo_move_enable(False) failed: {result}")
    except KeyboardInterrupt:
        print("已中断;伺服模式已退出。确认机械臂已停止。")
        raise SystemExit(130) from None


if __name__ == "__main__":
    main()
