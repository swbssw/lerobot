"""Log the SO leader's six servos during motion to find which joint lags.

Connects to the leader arm (torque stays disabled, so it stays backdrivable), reads
all six joint positions at high rate while YOU move the arm, then reports the last
time each joint was still moving. If one joint (e.g. shoulder_lift) keeps moving
long after the others have stopped, that joint is the source of the teleop lag.

Usage:
  1. Make sure teleoperate.py is NOT running (the port can only be used by one process).
  2. Run this script.
  3. Hold the arm still for ~2 s, then move it around (lift/rotate/swing) for ~5 s,
     then hold it still until the script ends.
"""

import argparse
import csv
import math
import time

from lerobot.teleoperators.so_leader import SOLeader, SOLeaderTeleopConfig

MOTOR_NAMES = ("shoulder_pan", "heavy_joint", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
MOVE_THRESHOLD_DEG_S = 1.0  # 高于这个速度算"还在动"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM8", help="主臂串口")
    parser.add_argument("--id", default="so101_jaka_leader", help="主臂标定 id")
    parser.add_argument("--duration-s", type=float, default=15.0, help="记录时长(秒)")
    parser.add_argument("--rate-hz", type=float, default=100.0, help="采样率")
    parser.add_argument("--output", default="leader_log.csv", help="日志文件")
    args = parser.parse_args()

    leader = SOLeader(SOLeaderTeleopConfig(port=args.port, id=args.id, use_degrees=True))
    print("连接主臂(不重新标定、不启动电机力矩,舵机保持回驱)...")
    leader.connect(calibrate=False)

    print(f"开始记录 {args.duration_s} 秒。")
    print("请:先静止 2 秒 → 来回移动主臂约 5 秒 → 最后保持静止直到结束。")
    print()

    period = 1.0 / args.rate_hz
    samples: list[tuple[float, ...]] = []
    start = time.monotonic()
    next_echo = start + 1.0
    try:
        while True:
            now = time.monotonic()
            t = now - start
            if t >= args.duration_s:
                break
            action = leader.get_action()
            row = (t,) + tuple(float(action[f"{m}.pos"]) for m in MOTOR_NAMES)
            samples.append(row)
            if now >= next_echo:
                print(f"  已记录 {t:5.1f} 秒 ...")
                next_echo += 1.0
            remaining = period - (time.monotonic() - now)
            if remaining > 0:
                time.sleep(remaining)
    finally:
        leader.disconnect()

    with open(args.output, "w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(("t_s",) + MOTOR_NAMES)
        writer.writerows(samples)
    print(f"\n原始数据已存到 {args.output}({len(samples)} 个采样点)")

    if len(samples) < 3:
        print("采样点太少,无法分析。请把记录时长调长一点重试。")
        return

    # 每个舵机:计算速度,找"最后一次还在动"的时间
    last_move: dict[str, float] = {}
    for idx, name in enumerate(MOTOR_NAMES):
        last = 0.0
        for a, b in zip(samples, samples[1:]):
            dt = b[0] - a[0]
            if dt <= 0:
                continue
            vel = (b[idx + 1] - a[idx + 1]) / dt
            if abs(vel) > MOVE_THRESHOLD_DEG_S:
                last = b[0]
        last_move[name] = last

    print("\n各舵机'最后停止运动'的时间(秒,越晚 = 越滞后):")
    for name in MOTOR_NAMES:
        marker = ""
        if name == "shoulder_lift":
            marker = "   ← 对应 JAKA J3"
        print(f"  {name:<14} {last_move[name]:6.2f} 秒{marker}")


if __name__ == "__main__":
    main()
