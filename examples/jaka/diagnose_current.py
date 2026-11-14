r"""实时读取 JAKA 每个关节的瞬时电流 / 力矩 / 温度 / 速度,并记录到 CSV。

直接在本机的 jkrc 兼容 Python(3.10,conda env `jaka`)下运行,不经过 lerobot 桥。
连接并登录后,每隔一段时间读一次 get_robot_status(),打印并保存 6 个关节的:
  - 瞬时电流 instCurrent(安培 A)
  - 瞬时力矩 instTorq(牛·米 Nm)
  - 温度 instTemperature(摄氏度 ℃)
  - 位置 joint_position(度,由弧度换算,用于对照姿态)
外加控制器柜温度 cabTemperature 和机器人平均电流 robotAverageCurrent。

关于"电流上限":JAKA SDK **不提供**电机峰值电流阈值的读取接口(它是控制器固件/硬件
参数,只能在 JAKA App 或 Zu 7 说明书里看,SDK 里搜不到)。所以本脚本只读**实际电流**,
你可以在快速运动时观察电流是不是顶到某个天花板(平台化),从而判断有没有限流。

安全:本脚本只读状态(login + get_robot_status),不命令运动、不进伺服模式。运行时请
**不要同时跑 teleoperate.py**(它们会抢同一个 SDK 连接)。想在运动中看电流,可以:
  1) 让 JAKA 进入拖动模式后手动拖臂;或 2) 在别的进程跑遥操时别用本脚本(见下)。

用法(在 lerobot 目录下):
  & "C:\Users\Administrator\miniconda3\envs\jaka\python.exe" examples\jaka\diagnose_current.py
  可选 --interval-s 0.5 --duration-s 30 --ip 10.5.5.100
"""

import argparse
import csv
import os
import sys
import time

JOINT_NAMES = ("J1_shoulder_pan", "J2_heavy_joint", "J3_shoulder_lift",
               "J4_elbow_flex", "J5_wrist_flex", "J6_wrist_roll")

# get_robot_status() 返回 (errno, status);status 是 25 元组(按 RobotStatus 结构体顺序)。
# 实测(2026-10 探针)字段位置:
IDX_JOINT_POSITION = 19          # status[19] = joint_position[6],弧度
IDX_ROBOT_MONITOR = 20           # status[20] = robot_monitor_data
# robot_monitor_data 内:
MON_CAB_TEMP = 2                 # 控制器柜温度(℃)
MON_AVG_CURRENT = 4              # 机器人平均电流(A)
MON_JOINT_MONITOR = 5            # jointMonitorData[6]
# 每个 jointMonitorData[j] = (instCurrent, instVoltage, instTemperature, instVel, instTorq)
JM_CURRENT = 0                   # 瞬时电流(A)
JM_VOLTAGE = 1                   # 瞬时电压(V)
JM_TEMPERATURE = 2               # 瞬时温度(℃)
JM_VELOCITY = 3                  # 瞬时速度(rad/s,控制器 1.7.0.20 以上)
JM_TORQUE = 4                    # 瞬时力矩(Nm)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip", default="10.5.5.100")
    parser.add_argument("--sdk-dir", default=r"E:\A_practice3\jaka_sdk\Windows\python3\x64")
    parser.add_argument("--interval-s", type=float, default=0.5, help="采样间隔(秒)")
    parser.add_argument("--duration-s", type=float, default=None,
                        help="记录时长(秒);省略则一直记录到 Ctrl+C")
    parser.add_argument("--output", default="current_log.csv", help="输出 CSV")
    args = parser.parse_args()

    if not 0 < args.interval_s <= 10:
        parser.error("--interval-s 需在 (0, 10] 之间")

    if hasattr(os, "add_dll_directory"):
        os.add_dll_directory(args.sdk_dir)
    sys.path.insert(0, args.sdk_dir)
    import jkrc

    rc = jkrc.RC(args.ip)
    login = rc.login()
    if not (isinstance(login, (tuple, list)) and login and login[0] == 0):
        raise SystemExit(f"登录失败:{login!r}")
    print(f"已连接 {args.ip}。开始每 {args.interval_s} 秒读一次电流/力矩/温度(只读,不运动)。")
    print("按 Ctrl+C 停止。\n")

    rows: list[list[float]] = []
    start = time.monotonic()
    next_report = start
    try:
        while args.duration_s is None or time.monotonic() - start < args.duration_s:
            now = time.monotonic()
            if now < next_report:
                time.sleep(next_report - now)
                continue
            next_report += args.interval_s

            ret = rc.get_robot_status()
            if not (isinstance(ret, (tuple, list)) and ret and ret[0] == 0):
                print(f"读取失败:{ret!r}")
                continue
            status = ret[1]
            joint_monitor = status[IDX_ROBOT_MONITOR][MON_JOINT_MONITOR]
            joint_pos_rad = status[IDX_JOINT_POSITION]
            cab_temp = status[IDX_ROBOT_MONITOR][MON_CAB_TEMP]
            avg_current = status[IDX_ROBOT_MONITOR][MON_AVG_CURRENT]

            t = now - start
            row = [t]
            currents = [joint_monitor[j][JM_CURRENT] for j in range(6)]
            torques = [joint_monitor[j][JM_TORQUE] for j in range(6)]
            temps = [joint_monitor[j][JM_TEMPERATURE] for j in range(6)]
            row += currents + torques + temps + list(joint_pos_rad)
            row += [avg_current, cab_temp]
            rows.append(row)

            # 一行打印:电流(A) + 力矩(Nm)
            cur_str = "  ".join(f"{c:+7.2f}" for c in currents)
            tor_str = "  ".join(f"{v:+7.2f}" for v in torques)
            tmp_str = "  ".join(f"{v:5.1f}" for v in temps)
            print(f"t={t:5.1f}s")
            print(f"  电流(A): {cur_str}")
            print(f"  力矩Nm : {tor_str}")
            print(f"  温度C  : {tmp_str}   控制器柜温 {cab_temp:.1f}C  平均电流 {avg_current:.2f}A")
    except KeyboardInterrupt:
        print("\n已停止。")
    finally:
        try:
            rc.logout()
        except Exception as exc:
            print(f"(logout 报错可忽略:{exc})")

    if not rows:
        print("没有采样点。")
        return

    columns = ["t_s"]
    columns += [f"{n}_cur_A" for n in JOINT_NAMES]
    columns += [f"{n}_torq_Nm" for n in JOINT_NAMES]
    columns += [f"{n}_temp_C" for n in JOINT_NAMES]
    columns += [f"{n}_pos_rad" for n in JOINT_NAMES]
    columns += ["robot_avg_current_A", "cab_temp_C"]
    with open(args.output, "w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(columns)
        writer.writerows(rows)

    print(f"\n原始数据已存到 {args.output}({len(rows)} 帧)")

    # 汇总:每个关节的最大 |电流|、|力矩|、温度峰值、由位置差估算的峰值速度
    n = len(rows)
    print(f"\n==== 汇总(共 {n} 帧)====")
    print(f"{'关节':<16} {'最大|电流|A':>11} {'最大|力矩|Nm':>12} {'峰值温度C':>10} {'峰值速度度/s':>12}")
    for j in range(6):
        cur = [abs(r[1 + j]) for r in rows]
        torq = [abs(r[1 + 6 + j]) for r in rows]
        temp = [r[1 + 12 + j] for r in rows]
        pos = [r[1 + 18 + j] for r in rows]
        tcol = [r[0] for r in rows]
        peak_vel = 0.0
        for i in range(1, n):
            dt = tcol[i] - tcol[i - 1]
            if dt > 0:
                peak_vel = max(peak_vel, abs(pos[i] - pos[i - 1]) / dt)
        peak_vel_deg = peak_vel * 180.0 / 3.141592653589793
        print(f"{JOINT_NAMES[j]:<16} {max(cur):>11.2f} {max(torq):>12.2f} {max(temp):>10.1f} {peak_vel_deg:>12.1f}")


if __name__ == "__main__":
    main()
