"""SO-ARM101 to JAKA teleoperation. The servo loop only sends leader targets; JAKA
joint angles are read back from the controller and printed periodically
(see --angle-interval-s)."""

import argparse
import logging
import math
import os
import time
from collections.abc import Sequence

from lerobot.robots.jaka_zu7_follower import JAKAZu7Follower, JAKAZu7FollowerConfig
from lerobot.teleoperators.so_leader import SOLeader, SOLeaderTeleopConfig

if __package__:
    from . import _system
    from .gravity_comp import GravityCompensator, GravityModel, PITCH_JOINTS
else:
    import _system
    from gravity_comp import GravityCompensator, GravityModel, PITCH_JOINTS

logger = logging.getLogger(__name__)

# JAKA Zu 7 joint names in the same order as the SO-ARM101 leader (J1..J6).
JAKA_ZU7_JOINTS = (
    "shoulder_pan",
    "heavy_joint",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)

_SETTINGS_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "teleop_settings.txt")


def _load_settings() -> dict[str, str]:
    """Read teleop_settings.txt (key = value) so values can be tuned without editing code."""
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
    """Apply teleop_settings.txt values as argparse defaults; CLI flags still override."""
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
        if "leader-cutoff-hz" in raw:
            defaults["leader_cutoff_hz"] = float(raw["leader-cutoff-hz"])
        if "leader-deadband-deg" in raw:
            values = tuple(float(value) for value in raw["leader-deadband-deg"].split())
            if len(values) not in (1, len(JAKA_ZU7_JOINTS)):
                raise ValueError("leader-deadband-deg 需要 1 个或 6 个角度值(J1-J6 顺序)")
            defaults["leader_deadband_deg"] = values
        if "angle-interval-s" in raw:
            defaults["angle_interval_s"] = float(raw["angle-interval-s"])
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


def _format_angle_line(joints_deg: list[float]) -> str:
    """One readable line: the six joint angles in degrees (logical/leader space)."""
    parts = [
        f"{name}={value:+8.3f}°"
        for name, value in zip(JAKA_ZU7_JOINTS, joints_deg, strict=True)
    ]
    return "关节角(deg) " + "  ".join(parts)


def _report_joint_angles(follower: JAKAZu7Follower) -> None:
    """Read back and print the JAKA joint angles; a failed read never stops teleop."""
    try:
        joints_deg = follower.get_joint_positions_deg()
    except Exception as exc:
        logger.warning("JAKA 关节角读取失败(忽略,继续摇操):%s", exc)
        return
    logger.info("%s", _format_angle_line(joints_deg))


def _max_joint_span(samples: list[dict[str, float]]) -> tuple[str, float]:
    if not samples:
        return "-", 0.0
    keys = (f"{joint}.pos" for joint in JAKA_ZU7_JOINTS)
    return max(
        (
            (key.removesuffix(".pos"), max(sample[key] for sample in samples) - min(sample[key] for sample in samples))
            for key in keys
            if key in samples[0]
        ),
        key=lambda item: item[1],
        default=("-", 0.0),
    )


def _setup_gravity_comp(leader: SOLeader, args: argparse.Namespace) -> GravityCompensator:
    """加载重力模型、给主臂四个俯仰关节上电并使能，返回运行时补偿器(复用 leader 的串口总线)。

    偏移模式(stiffness>0)让舵机输出恒定托举力、随位姿跟踪、无固定锚点：
    松手停住、不下坠、不回原位、掰动轻。过补偿(P>stiffness)是托住模型残差的关键。
    """
    model_path = args.gravity_model
    if not os.path.isfile(model_path):
        # 若从别的目录启动,则相对脚本所在目录(examples/jaka)再找一次
        alt = os.path.join(os.path.dirname(os.path.abspath(__file__)), model_path)
        if os.path.isfile(alt):
            model_path = alt
    model = GravityModel.load(model_path)
    joints = list(PITCH_JOINTS)
    jrmse = {j: round(model.joint_rmse.get(j, 0.0), 1) for j in joints}
    logger.info("加载重力模型 %s(%d 样本, 各关节 RMSE=%s)", args.gravity_model, model.n_samples, jrmse)
    # 上电前把 Goal 设为当前位置，避免突然回弹
    pos = leader.bus.sync_read("Present_Position", joints, normalize=False)
    leader.bus.sync_write("Goal_Position", pos, normalize=False)
    # 写舵机 P/I 增益(过补偿 P>stiffness 才托得住；I 保持 0 避免对恒定偏移积分缓爬)
    if args.gravity_p is not None:
        for j in joints:
            leader.bus.write("P_Coefficient", j, args.gravity_p, normalize=False)
    if args.gravity_i is not None:
        for j in joints:
            leader.bus.write("I_Coefficient", j, args.gravity_i, normalize=False)
    leader.bus.enable_torque(joints)
    margins = {
        "heavy_joint": args.gravity_margin_heavy_joint,
        "shoulder_lift": args.gravity_margin_shoulder_lift,
        "elbow_flex": args.gravity_margin_elbow_flex,
        "wrist_flex": args.gravity_margin_wrist_flex,
    }
    stiffness = {
        "heavy_joint": args.gravity_stiffness,
        "shoulder_lift": args.gravity_stiffness_shoulder_lift,
        "elbow_flex": args.gravity_stiffness_elbow_flex,
        "wrist_flex": args.gravity_stiffness_wrist_flex,
    }
    comp = GravityCompensator(
        leader.bus,
        model,
        margin=margins,
        stiffness=stiffness,
        anti_droop=args.gravity_anti_droop,
        anti_droop_gain=args.gravity_anti_droop_gain,
        anti_droop_kp=args.gravity_anti_droop_kp,
        anti_droop_threshold=args.gravity_anti_droop_threshold,
        anti_droop_max=args.gravity_anti_droop_max,
    )
    logger.info(
        "主臂重力补偿已启用:P=%d stiffness=%s margin=%s anti-droop=%s",
        args.gravity_p, stiffness, margins, args.gravity_anti_droop,
    )
    return comp


def _joint_deadbands(deadband_deg: float | Sequence[float]) -> dict[str, float]:
    if isinstance(deadband_deg, (int, float)):
        values = (float(deadband_deg),)
    else:
        values = tuple(float(value) for value in deadband_deg)
    if len(values) == 1:
        values *= len(JAKA_ZU7_JOINTS)
    if len(values) != len(JAKA_ZU7_JOINTS) or any(not math.isfinite(v) or v < 0 for v in values):
        raise ValueError("leader deadband requires one or six finite, non-negative angles (J1-J6 order)")
    return {f"{joint}.pos": float(value) for joint, value in zip(JAKA_ZU7_JOINTS, values, strict=True)}


class LeaderTargetFilter:
    """Suppress encoder jitter without shifting the first relative-start target."""

    def __init__(self, cutoff_hz: float, deadband_deg: float | Sequence[float] = 0.0):
        if not math.isfinite(cutoff_hz) or cutoff_hz <= 0:
            raise ValueError("leader cutoff must be finite and positive")
        self.cutoff_hz = cutoff_hz
        self.deadband_deg = _joint_deadbands(deadband_deg)
        self._last_time: float | None = None
        self._last_action: dict[str, float] = {}

    def apply(self, action: dict[str, float], now: float) -> dict[str, float]:
        if self._last_time is None:
            self._last_time = now
            self._last_action = dict(action)
            return dict(action)
        dt = max(0.0, now - self._last_time)
        blend = -math.expm1(-2 * math.pi * self.cutoff_hz * dt)
        filtered = {}
        for key, value in action.items():
            previous = self._last_action.get(key, value)
            delta = value - previous
            # 临时禁用死区以对比遥操效果；恢复时取消下面两行注释。
            # if key in self.deadband_deg:
            #     delta = math.copysign(max(0.0, abs(delta) - self.deadband_deg[key]), delta)
            filtered[key] = previous + blend * delta
        self._last_time = now
        self._last_action = filtered
        return filtered


def run_loop(
    leader: SOLeader,
    follower: JAKAZu7Follower,
    fps: float,
    duration_s: float | None = None,
    angle_interval_s: float | None = None,
    comp: GravityCompensator | None = None,
    leader_cutoff_hz: float = 0.0,
    leader_deadband_deg: float | Sequence[float] = 0.0,
) -> None:
    """Read the latest leader target, send once, and pace; never substitute targets for feedback."""
    if not math.isfinite(fps) or fps <= 0:
        raise ValueError("fps must be finite and positive")
    if duration_s is not None and (not math.isfinite(duration_s) or duration_s <= 0):
        raise ValueError("duration_s must be finite and positive")
    start = report_start = time.monotonic()
    next_angle = start + angle_interval_s if angle_interval_s is not None else None
    period = 1.0 / fps
    target_filter = LeaderTargetFilter(leader_cutoff_hz, leader_deadband_deg) if leader_cutoff_hz else None
    ticks = 0
    read_total = send_total = worst_work = 0.0
    first_action: dict[str, float] | None = None
    first_command: dict[str, float] | None = None
    raw_samples: list[dict[str, float]] = []
    target_samples: list[dict[str, float]] = []
    command_samples: list[dict[str, float]] = []
    while duration_s is None or time.monotonic() - start < duration_s:
        tick = time.monotonic()
        raw_action = leader.get_action()
        action = raw_action
        if target_filter is not None:
            action = target_filter.apply(action, tick)
        if comp is not None:
            comp.step()
        after_read = time.monotonic()
        sent = follower.send_action(action)
        after_send = time.monotonic()
        raw_samples.append(dict(raw_action))
        target_samples.append(dict(action))
        command_samples.append(dict(sent))
        if first_action is None or first_command is None:
            first_action = {key: float(action[key]) for key in sent}
            first_command = {key: float(value) for key, value in sent.items()}
        target_gap = max(
            abs(first_command[key] + float(action[key]) - first_action[key] - float(sent[key]))
            for key in sent
            if key != "gripper.pos"
        )
        ticks += 1
        read_total += after_read - tick
        send_total += after_send - after_read
        worst_work = max(worst_work, after_send - tick)
        if after_send - report_start >= 1.0:
            logger.info(
                "%.1f Hz | leader read %.2f ms | SDK send %.2f ms | worst work %.2f ms | "
                "target-command gap %.2f deg (not measured robot feedback)",
                ticks / (after_send - report_start),
                1000 * read_total / ticks,
                1000 * send_total / ticks,
                1000 * worst_work,
                target_gap,
            )
            raw_joint, raw_span = _max_joint_span(raw_samples)
            target_joint, target_span = _max_joint_span(target_samples)
            command_joint, command_span = _max_joint_span(command_samples)
            logger.info(
                "1s motion span: leader %s %.2f deg | filtered %s %.2f deg | "
                "JAKA command %s %.2f deg (not measured feedback)",
                raw_joint, raw_span, target_joint, target_span, command_joint, command_span,
            )
            raw_samples.clear()
            target_samples.clear()
            command_samples.clear()
            if comp is not None:
                _corr = comp.corr
                logger.info(
                    "  重力修正 corr  heavy=%.0f shoulder=%.0f elbow=%.0f wrist=%.0f",
                    _corr["heavy_joint"], _corr["shoulder_lift"], _corr["elbow_flex"], _corr["wrist_flex"],
                )
            report_start = after_send
            ticks = 0
            read_total = send_total = worst_work = 0.0
        if next_angle is not None and after_send >= next_angle:
            _report_joint_angles(follower)
            # Skip any windows we fell behind on; never burst-read to catch up.
            while next_angle <= after_send:
                next_angle += angle_interval_s
        # Missed deadlines never enqueue catch-up frames or old leader targets.
        remaining = period - (time.monotonic() - tick)
        if remaining > 0:
            time.sleep(remaining)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip", default="10.5.5.100")
    parser.add_argument("--sdk-python", default=None, help="SDK 解释器(默认按所选系统填入)")
    parser.add_argument("--sdk-dir", default=None, help="jkrc SDK 目录(默认按所选系统填入)")
    parser.add_argument("--port", default=None, help="主臂串口(默认按所选系统填入)")
    parser.add_argument("--leader-id", default="so101_jaka_leader")
    parser.add_argument("--fps", type=float, default=125)
    parser.add_argument("--leader-cutoff-hz", type=float, default=12.0,
                        help="主臂角度低通截止频率(Hz); 0 禁用,不影响 JAKA 速度上限")
    parser.add_argument("--leader-deadband-deg", type=float, nargs="+",
                        default=(0.2, 2.0, 2.0, 0.5, 0.1, 0.1),
                        help="主臂角度软死区(度);填 1 个值统一设置,或按 J1-J6 顺序填 6 个值")
    parser.add_argument("--servo-step-num", type=int, default=1)
    parser.add_argument("--servo-api", choices=("servo_j", "servo_j_extend"), default="servo_j")
    parser.add_argument("--max-velocity-deg-s", type=float, default=100)
    parser.add_argument("--max-acceleration-deg-s2", type=float, default=600)
    parser.add_argument("--max-jerk-deg-s3", type=float, default=2000)
    parser.add_argument("--joint-signs", type=int, nargs=6, default=[-1, 1, 1, 1, -1, 1])
    parser.add_argument("--gripper-can-channel", default=None, help="CAN 接口名(can0 或已确认兼容 SLCAN 的串口)")
    parser.add_argument("--gripper-can-interface", choices=("socketcan", "slcan", "dm_usb2can"), default=None)
    parser.add_argument("--gripper-can-id", type=lambda value: int(value, 0), default=None)
    parser.add_argument("--gripper-feedback-id", type=lambda value: int(value, 0), default=None)
    parser.add_argument("--gripper-open-deg", type=float, default=None)
    parser.add_argument("--gripper-closed-deg", type=float, default=None)
    parser.add_argument("--gripper-max-velocity-deg-s", type=float, default=150.0)
    parser.add_argument("--gripper-leader-sign", type=int, choices=(-1, 1), default=1,
                        help="主臂夹爪合拢时读数减小选 -1，增大选 1")
    parser.add_argument("--gripper-control-mode", type=int, choices=(2, 4), default=2,
                        help="2 位置速度模式；4 力位混合模式（需先在达妙工具设置）")
    parser.add_argument("--gripper-current-limit-pct", type=float, default=None,
                        help="模式4的最大相电流百分比，不是夹爪处的力（N）")
    parser.add_argument("--duration-s", type=float)
    parser.add_argument(
        "--angle-interval-s",
        type=float,
        default=2.0,
        help="每隔多少秒读一次 JAKA 各关节角度并打印;设 0 关闭",
    )
    parser.add_argument("--configure-nlf", action=argparse.BooleanOptionalAction, default=True)
    # 主臂重力补偿(失重):提供 --gravity-model 才启用
    parser.add_argument("--gravity-model", default=None, help="主臂重力模型路径(如 gravity_model.json);提供即启用重力补偿")
    parser.add_argument("--gravity-p", type=int, default=60, help="重力补偿舵机 P 增益(过补偿,默认 60;越小越软越易下垂)")
    parser.add_argument("--gravity-i", type=int, default=0, help="重力补偿舵机 I 增益(默认 0,保持清零)")
    parser.add_argument("--gravity-stiffness", type=float, default=32.0, help="重力补偿偏移分母(=舵机基础 P,默认 32)")
    parser.add_argument("--gravity-stiffness-shoulder-lift", type=float, default=26.0,
                        help="ID 3 单独的补偿偏移分母(默认 26;越小托举越强)")
    parser.add_argument("--gravity-stiffness-elbow-flex", type=float, default=30.0,
                        help="ID 4 单独的补偿偏移分母(默认 30)")
    parser.add_argument("--gravity-stiffness-wrist-flex", type=float, default=30.0,
                        help="ID 6 单独的补偿偏移分母(默认 30)")
    parser.add_argument("--gravity-margin-heavy-joint", type=float, default=45.0)
    parser.add_argument("--gravity-margin-shoulder-lift", type=float, default=160.0,
                        help="ID 3 力矩上限的余量(默认 160)")
    parser.add_argument("--gravity-margin-elbow-flex", type=float, default=55.0)
    parser.add_argument("--gravity-margin-wrist-flex", type=float, default=50.0)
    parser.add_argument("--gravity-anti-droop", action=argparse.BooleanOptionalAction, default=True, help="启用抗下垂积分器")
    parser.add_argument("--gravity-anti-droop-gain", type=float, default=1.0)
    parser.add_argument("--gravity-anti-droop-kp", type=float, default=30.0)
    parser.add_argument("--gravity-anti-droop-threshold", type=float, default=5.0)
    parser.add_argument("--gravity-anti-droop-max", type=float, default=400.0)
    _apply_settings(parser)
    args = parser.parse_args()
    if args.gripper_can_channel is not None:
        if args.gripper_open_deg is None:
            args.gripper_open_deg = 0.0
        if args.gripper_closed_deg is None:
            args.gripper_closed_deg = 120.0
        if args.gripper_control_mode == 4 and args.gripper_current_limit_pct is None:
            args.gripper_current_limit_pct = 1.61
    if not math.isfinite(args.fps) or args.fps <= 0:
        parser.error("--fps must be finite and positive")
    if not math.isfinite(args.leader_cutoff_hz) or args.leader_cutoff_hz < 0:
        parser.error("--leader-cutoff-hz must be finite and non-negative")
    try:
        _joint_deadbands(args.leader_deadband_deg)
    except ValueError as exc:
        parser.error(str(exc))
    if args.duration_s is not None and (not math.isfinite(args.duration_s) or args.duration_s <= 0):
        parser.error("--duration-s must be finite and positive")
    if not math.isfinite(args.angle_interval_s):
        parser.error("--angle-interval-s must be finite")
    for name in ("gravity_stiffness_shoulder_lift", "gravity_stiffness_elbow_flex", "gravity_stiffness_wrist_flex"):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f"--{name.replace('_', '-')} must be finite and positive")
    for name in ("gravity_margin_shoulder_lift", "gravity_margin_elbow_flex", "gravity_margin_wrist_flex"):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) < 0:
            parser.error(f"--{name.replace('_', '-')} must be finite and non-negative")
    angle_interval_s = args.angle_interval_s if args.angle_interval_s > 0 else None
    defaults = _system.choose()
    args.sdk_python = args.sdk_python or defaults["sdk_python"]
    args.sdk_dir = args.sdk_dir or defaults["sdk_dir"]
    args.port = args.port or defaults["port"]
    # force=True: importing lerobot already attached a handler to the root logger,
    # which makes a plain basicConfig() a no-op and silently swallows every INFO log.
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s", force=True)
    logger.info("Actual leader port: %s | gripper: %s (%s)",
                args.port, args.gripper_can_channel or "disabled", args.gripper_can_interface or "none")
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
            gripper_can_interface=args.gripper_can_interface,
            gripper_can_id=args.gripper_can_id,
            gripper_feedback_id=args.gripper_feedback_id,
            gripper_open_deg=args.gripper_open_deg,
            gripper_closed_deg=args.gripper_closed_deg,
            gripper_max_velocity_deg_s=args.gripper_max_velocity_deg_s,
            gripper_leader_sign=args.gripper_leader_sign,
            gripper_control_mode=args.gripper_control_mode,
            gripper_current_limit_pct=args.gripper_current_limit_pct,
        )
    )
    logger.info(
        "Stop other robot-control programs. Robot must already be powered and enabled. "
        "Hold the leader still until connected. Initial poses become the relative reference; "
        "no reset move is sent."
    )
    comp: GravityCompensator | None = None
    try:
        leader.connect()
        if args.gravity_model:
            comp = _setup_gravity_comp(leader, args)
        # Start CAN command streaming immediately after enabling the gripper;
        # do not spend its communication timeout setting up leader compensation.
        follower.connect()
        logger.info(
            "Teleoperation starting: %.1f Hz, speed cap %.1f deg/s. Ctrl+C to stop.",
            args.fps,
            args.max_velocity_deg_s,
        )
        if angle_interval_s is not None:
            logger.info("关节角监视已开启:每 %.1f 秒打印一次各关节角度。", angle_interval_s)
        run_loop(leader, follower, args.fps, args.duration_s, angle_interval_s, comp,
                 args.leader_cutoff_hz, args.leader_deadband_deg)
    except KeyboardInterrupt:
        logger.info("Stopping teleoperation.")
    finally:
        # Always release JAKA streaming control even if leader cleanup fails.
        try:
            follower.disconnect()
        finally:
            if leader.is_connected:
                if comp is not None:
                    try:
                        # 退出前给主臂俯仰关节断电，让手臂放松(否则会一直保持托举锁定)
                        leader.bus.disable_torque(list(PITCH_JOINTS))
                    except Exception:
                        pass
                leader.disconnect()


if __name__ == "__main__":
    main()
