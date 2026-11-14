"""遥操时记录 JAKA 从臂和 4340 夹爪的实时反馈状态。

每个采样帧只包含从臂反馈,不保存主臂读数、发送目标或误差:
  - JAKA 六个关节实际角度(度)
  - JAKA 六个关节实时电流、力矩、温度和速度
  - 4340 夹爪实际位置(度和 0-100% 百分比)

主臂仍用于产生遥操动作,但不会写入 CSV。JAKA 角度和关节监视数据来自
控制器反馈;夹爪位置来自 4340 的实际位置寄存器。结束(或 Ctrl+C)后写入 CSV。

安全:机械臂必须先上电并使能;主臂在连接时保持静止(首帧即相对参考,不发复位动作);
保持急停可及,感觉异样立刻 Ctrl+C。

用法:
  1. 确认 teleoperate.py 没在跑(串口只能被一个进程占用)。
  2. 机械臂上电、使能。
  3. 运行本脚本(默认跑 15 秒,也可 --duration-s 0 改成 Ctrl+C 结束)。
  4. 先静止约 2 秒,再自然地来回移动主臂,最后静止直到结束。
"""

import argparse
import csv
import logging
import math
import os
import time

from lerobot.robots.jaka_zu7_follower import JAKAZu7Follower, JAKAZu7FollowerConfig
from lerobot.teleoperators.so_leader import SOLeader, SOLeaderTeleopConfig

import _system  # sibling module: interactive OS selection

logger = logging.getLogger(__name__)

JAKA_ZU7_JOINTS = (
    "shoulder_pan",
    "heavy_joint",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)
N_JOINTS = len(JAKA_ZU7_JOINTS)

_SETTINGS_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "teleop_settings.txt")


def _load_settings() -> dict[str, str]:
    """Read teleop_settings.txt (key = value), same as teleoperate.py."""
    settings: dict[str, str] = {}
    if not os.path.isfile(_SETTINGS_PATH):
        return settings
    with open(_SETTINGS_PATH, encoding="utf-8-sig", errors="ignore") as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, _, value = line.partition("=")
            settings[key.strip()] = value.strip()
    return settings


def _apply_settings(parser: argparse.ArgumentParser) -> None:
    raw = _load_settings()
    defaults: dict[str, object] = {}
    try:
        if "max-velocity-deg-s" in raw:
            defaults["max_velocity_deg_s"] = float(raw["max-velocity-deg-s"])
        if "max-acceleration-deg-s2" in raw:
            defaults["max_acceleration_deg_s2"] = float(raw["max-acceleration-deg-s2"])
        if "max-jerk-deg-s3" in raw:
            defaults["max_jerk_deg_s3"] = float(raw["max-jerk-deg-s3"])
        if "fps" in raw:
            defaults["fps"] = float(raw["fps"])
        if "joint-signs" in raw:
            signs = [int(x) for x in raw["joint-signs"].split()]
            if len(signs) != 6:
                raise ValueError("joint-signs 需要 6 个整数(如 -1 1 1 1 -1 1)")
            defaults["joint_signs"] = signs
        if "configure-nlf" in raw:
            defaults["configure_nlf"] = raw["configure-nlf"].strip().lower() in {
                "1", "true", "yes", "on", "enable", "enabled",
            }
    except ValueError as exc:
        raise SystemExit(f"teleop_settings.txt 格式错误:{exc}") from exc
    parser.set_defaults(**defaults)


def _median_dt(times: list[float]) -> float:
    if len(times) < 2:
        return 0.0
    diffs = sorted(b - a for a, b in zip(times, times[1:]) if b > a)
    if not diffs:
        return 0.0
    return diffs[len(diffs) // 2]


def _record_loop(leader: SOLeader, follower: JAKAZu7Follower, fps: float,
                 record_hz: float, duration_s: float | None, rows: list[list[float]]) -> None:
    period = 1.0 / fps
    record_period = 1.0 / record_hz
    start = time.monotonic()
    last_record = start - record_period  # 第一帧就记录
    next_echo = start + 1.0
    while duration_s is None or time.monotonic() - start < duration_s:
        tick = time.monotonic()
        action = leader.get_action()
        # The leader is only an input device here. Its values and the command
        # returned by send_action are deliberately not written to the dataset.
        follower.send_action(action)
        now = time.monotonic()
        if now - last_record >= record_period:
            obs = follower.get_observation()
            monitor = follower.get_joint_monitor()
            t = now - start
            row: list[float] = [t]
            for name in JAKA_ZU7_JOINTS:
                row.append(float(obs[f"{name}.pos"]))
            row += [v if v is not None else float("nan") for v in monitor["current_a"]]
            row += [v if v is not None else float("nan") for v in monitor["torque_nm"]]
            row += [v if v is not None else float("nan") for v in monitor["temperature_c"]]
            row += [v if v is not None else float("nan") for v in monitor["velocity_rad_s"]]
            gripper_pct = float(obs.get("gripper.pos", float("nan")))
            if (
                math.isfinite(gripper_pct)
                and follower.config.gripper_open_deg is not None
                and follower.config.gripper_closed_deg is not None
            ):
                gripper_deg = follower.config.gripper_open_deg + gripper_pct / 100.0 * (
                    follower.config.gripper_closed_deg - follower.config.gripper_open_deg
                )
            else:
                gripper_deg = float("nan")
            row += [gripper_deg, gripper_pct]
            rows.append(row)
            last_record = now
            if now >= next_echo:
                print(f"  已记录 {t:5.1f} 秒 ... JAKA/夹爪实际反馈已采集")
                next_echo += 1.0
        remaining = period - (time.monotonic() - tick)
        if remaining > 0:
            time.sleep(remaining)


def _write_csv(rows: list[list[float]], output: str) -> None:
    columns = ["t_s"]
    for name in JAKA_ZU7_JOINTS:
        columns.append(f"jaka_{name}_pos_deg")
    for name in JAKA_ZU7_JOINTS:
        columns.append(f"jaka_{name}_current_A")
    for name in JAKA_ZU7_JOINTS:
        columns.append(f"jaka_{name}_torque_Nm")
    for name in JAKA_ZU7_JOINTS:
        columns.append(f"jaka_{name}_temperature_C")
    for name in JAKA_ZU7_JOINTS:
        columns.append(f"jaka_{name}_velocity_rad_s")
    columns += ["gripper_actual_deg", "gripper_actual_pct"]
    with open(output, "w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(columns)
        writer.writerows(rows)
    print(f"\n原始数据已存到 {output}({len(rows)} 帧)")


def _summarize(rows: list[list[float]]) -> None:
    n = len(rows)
    times = [r[0] for r in rows]
    dt = _median_dt(times)
    span = times[-1] - times[0] if n else 0.0
    print(f"\n==== 记录结果(共 {n} 帧,约 {span:.1f} 秒,采样间隔约 {dt * 1000:.1f} ms)====")
    print("说明:CSV 中只有 JAKA 和 4340 的实际反馈状态;没有主臂、指令值或误差列。")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip", default="10.5.5.100")
    parser.add_argument("--sdk-python", default=None, help="SDK 解释器(默认按所选系统填入)")
    parser.add_argument("--sdk-dir", default=None, help="jkrc SDK 目录(默认按所选系统填入)")
    parser.add_argument("--port", default=None, help="主臂串口(默认按所选系统填入)")
    parser.add_argument("--leader-id", default="so101_jaka_leader")
    parser.add_argument("--fps", type=float, default=125, help="伺服指令频率")
    parser.add_argument("--record-hz", type=float, default=50, help="记录频率(读取 JAKA/4340 实际反馈)")
    parser.add_argument("--duration-s", type=float, default=15.0,
                        help="记录时长(秒);0 或省略则一直记录到 Ctrl+C")
    parser.add_argument("--output", default="record_both.csv", help="输出 CSV 文件")
    parser.add_argument("--servo-step-num", type=int, default=1)
    parser.add_argument("--servo-api", choices=("servo_j", "servo_j_extend"), default="servo_j")
    parser.add_argument("--max-velocity-deg-s", type=float, default=60)
    parser.add_argument("--max-acceleration-deg-s2", type=float, default=80)
    parser.add_argument("--max-jerk-deg-s3", type=float, default=240)
    parser.add_argument("--joint-signs", type=int, nargs=6, default=[-1, 1, 1, 1, -1, 1])
    parser.add_argument("--configure-nlf", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--gripper-can-channel", default=None, help="4340 USB2CAN串口;提供后记录夹爪反馈")
    parser.add_argument("--gripper-can-id", type=lambda value: int(value, 0), default=0x7)
    parser.add_argument("--gripper-feedback-id", type=lambda value: int(value, 0), default=0x14)
    parser.add_argument("--gripper-open-deg", type=float, default=0.0)
    parser.add_argument("--gripper-closed-deg", type=float, default=120.0)
    parser.add_argument("--gripper-max-velocity-deg-s", type=float, default=150.0)
    parser.add_argument("--gripper-leader-sign", type=int, choices=(-1, 1), default=-1)
    parser.add_argument("--gripper-control-mode", type=int, choices=(2, 4), default=4)
    parser.add_argument("--gripper-current-limit-pct", type=float, default=1.61)
    _apply_settings(parser)
    args = parser.parse_args()

    if args.gripper_can_channel is not None and args.gripper_control_mode == 4:
        if args.gripper_current_limit_pct is None:
            parser.error("模式4需要--gripper-current-limit-pct")

    if not math.isfinite(args.fps) or args.fps <= 0:
        parser.error("--fps 必须为正数")
    if not math.isfinite(args.record_hz) or args.record_hz <= 0:
        parser.error("--record-hz 必须为正数")
    duration_s = args.duration_s if args.duration_s and args.duration_s > 0 else None

    defaults = _system.choose()
    args.sdk_python = args.sdk_python or defaults["sdk_python"]
    args.sdk_dir = args.sdk_dir or defaults["sdk_dir"]
    args.port = args.port or defaults["port"]

    # force=True: importing lerobot already attached a handler to the root logger,
    # which makes a plain basicConfig() a no-op and silently swallows every INFO log.
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s", force=True)

    leader = SOLeader(SOLeaderTeleopConfig(port=args.port, id=args.leader_id, use_degrees=True))
    follower = JAKAZu7Follower(
        JAKAZu7FollowerConfig(
            ip_address=args.ip,
            sdk_python=args.sdk_python,
            sdk_dir=args.sdk_dir,
            sdk_timeout_s=15.0,
            servo_api=args.servo_api,
            servo_step_num=args.servo_step_num,
            relative_start=True,
            joint_signs=tuple(args.joint_signs),
            configure_servo_nlf=args.configure_nlf,
            max_joint_velocity_deg_s=args.max_velocity_deg_s,
            max_joint_acceleration_deg_s2=args.max_acceleration_deg_s2,
            max_joint_jerk_deg_s3=args.max_jerk_deg_s3,
            gripper_can_channel=args.gripper_can_channel,
            gripper_can_interface="dm_usb2can" if args.gripper_can_channel else None,
            gripper_can_id=args.gripper_can_id if args.gripper_can_channel else None,
            gripper_feedback_id=args.gripper_feedback_id if args.gripper_can_channel else None,
            gripper_open_deg=args.gripper_open_deg if args.gripper_can_channel else None,
            gripper_closed_deg=args.gripper_closed_deg if args.gripper_can_channel else None,
            gripper_max_velocity_deg_s=args.gripper_max_velocity_deg_s,
            gripper_leader_sign=args.gripper_leader_sign,
            gripper_control_mode=args.gripper_control_mode if args.gripper_can_channel else 2,
            gripper_current_limit_pct=(
                args.gripper_current_limit_pct if args.gripper_can_channel and args.gripper_control_mode == 4 else None
            ),
        )
    )

    print("注意:机械臂必须已上电并使能;主臂在连接时保持静止;保持急停可及。")
    rows: list[list[float]] = []
    try:
        leader.connect()
        follower.connect()
        print(f"开始记录(伺服 {args.fps:.0f} Hz,采样 {args.record_hz:.0f} Hz)。")
        print("先静止约 2 秒 → 自然移动主臂 → 最后静止直到结束;Ctrl+C 可随时停止。\n")
        _record_loop(leader, follower, args.fps, args.record_hz, duration_s, rows)
    except KeyboardInterrupt:
        print("\n已中断,正在收尾...")
    finally:
        try:
            follower.disconnect()
        except Exception as exc:
            # servo_move_enable(False) 返回 -61 是误报(伺服实际已退出),忽略即可
            logger.warning("从臂断开时的小报错(通常是 -61 误报,伺服已退出):%s", exc)
        if leader.is_connected:
            try:
                leader.disconnect()
            except Exception as exc:
                logger.warning("主臂断开报错:%s", exc)

    if rows:
        _write_csv(rows, args.output)
        _summarize(rows)
    else:
        print("\n没有记录到任何数据(可能连接后立刻中断)。请重试。")
        raise SystemExit(1)


if __name__ == "__main__":
    main()
