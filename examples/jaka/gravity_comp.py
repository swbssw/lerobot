#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Gravity compensation for the STS3215 SO-101 leader arm (主臂重力补偿).

为什么可行
----------
STS3215 没有力矩模式，但它有两个关键寄存器：

* ``Present_Load`` (地址 60)：当前输出力矩的反馈，有符号，量纲与下面一致，范围约 ±1000。
* ``Torque_Limit``  (地址 48)：位置模式下舵机能输出的最大力矩，无符号，范围 0-1000。

二者同量纲。所以：**在某个静止位姿下，舵机为了对抗重力读到的 load 值，
就是我们要写的 torque limit**（再加一点 margin 克服摩擦/建模误差）。

运行时只需要：
    1. 读各俯仰关节的当前位置 q；
    2. 用拟合出的重力模型算出每个关节需要的 |τ_g(q)|；
    3. 写 ``Torque_Limit = |τ_g(q)| + margin``。
方向由舵机内部 PID 自己负责。

模型（每关节独立拟合，共 24 系数：20 个 sin/cos + 4 个常数项）
------------------------------------------------------------------
四个俯仰关节（按 so_leader.py 里的 motor 名，沿运动链顺序）：

* ``heavy_joint``   = J2 上臂俯仰 (ID 2)
* ``shoulder_lift`` = J3 前臂俯仰 (ID 3)
* ``elbow_flex``    = J4 肘俯仰   (ID 4)
* ``wrist_flex``    = J5 腕俯仰   (ID 6)

绝对角（相对竖直方向）由原始编码器累加得到：

    α2 = s2·φ2
    α3 = s2·φ2 + s3·φ3
    α4 = s2·φ2 + s3·φ3 + s4·φ4
    α5 = s2·φ2 + s3·φ3 + s4·φ4 + s5·φ5

每个关节的重力矩只取决于它自己和下游各连杆的绝对角，用 sin+cos 基底：

    τ_J2 = Σ_{i=2..5} [P_i·sinα_i + Q_i·cosα_i] + C2   (9 系数)
    τ_J3 = Σ_{i=3..5} [P_i·sinα_i + Q_i·cosα_i] + C3   (7 系数)
    τ_J4 = Σ_{i=4..5} [P_i·sinα_i + Q_i·cosα_i] + C4   (5 系数)
    τ_J5 =             P_5·sinα5 + Q_5·cosα5   + C5   (3 系数)

每个关节的常数项 C_j 捕获摩擦/装配造成的静态 DC 偏移（sin/cos 均值为 0，无法表示它）。

α5（腕）在 J2/J3/J4 里确实存在（夹爪较重，经腕→前臂传到近端关节），但不是主导项。
旧版随机采样时 α4/α5 强相关，最小二乘把肘的方差错分给了腕（α5 系数虚高到与 α4 相当）；
系统扫掠把 φ5 独立出来后才解耦，现 α4 幅度 > α5 幅度（J2 39>24、J3 52>33、J4 52>33）。

每个关节独立做最小二乘，**不强制跨关节共享系数**。原因：实测发现若强制
"前臂对 J2/J3 用同一系数"，J3 的增益会被 J2 拉低约 25%（实测 J3 到 -296，
共享模型数学上最多只能到 ~243），说明这条臂的肩部不是严格串联链。独立
拟合对机械结构更鲁棒。
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

# 四个需要重力补偿的俯仰关节（与 so_leader.py 里的 motor 名一致，沿运动链顺序）
PITCH_JOINTS: tuple[str, str, str, str] = ("heavy_joint", "shoulder_lift", "elbow_flex", "wrist_flex")

# 四个绝对角的名字（依次对应 PITCH_JOINTS 里各关节驱动的连杆）
ANGLE_NAMES = ("α2", "α3", "α4", "α5")

# 每个关节依赖多少个下游绝对角（含自己）。
# shoulder_lift 用 α3+α4+α5：α3 前臂主导项，α4 肘伸开时腕/夹爪经肘连杆长作用在 J3，
# α5 腕+夹爪自身姿态（夹爪较重，经前臂传到近端关节，真实但非主导）。系统扫掠已把
# α4/α5 解耦（旧版随机采样强相关导致 α5 被虚高），现 α4 幅度 > α5 幅度。
JOINT_NUM_ANGLES: dict[str, int] = {
    "heavy_joint": 4,   # α2,α3,α4,α5（近端关节，下游都经长力臂作用）
    "shoulder_lift": 3,  # α3,α4,α5
    "elbow_flex": 2,    # α4,α5
    "wrist_flex": 1,    # α5
}

# STS3215 分辨率与每步弧度
RESOLUTION = 4096.0
RAD_PER_STEP = 2.0 * math.pi / RESOLUTION

MAX_TORQUE_LIMIT = 1000.0


@dataclass
class GravityModel:
    """拟合好的重力补偿模型（每关节独立系数）。"""

    signs: tuple[int, int, int, int]  # (s2, s3, s4, s5)
    coeffs: dict[str, list[float]]  # joint -> [sin, cos, sin, cos, ...]（下游各绝对角）
    intercepts: dict[str, float] = field(default_factory=dict)  # joint -> 常数项(摩擦/装配 DC)
    rmse: float = 0.0  # 联合拟合残差 RMSE（load 量纲）
    n_samples: int = 0
    joint_rmse: dict[str, float] = field(default_factory=dict)  # 各关节单独 RMSE

    # ------------------------------------------------------------------
    # 预测
    # ------------------------------------------------------------------
    @staticmethod
    def _abs_angles(p2: float, p3: float, p4: float, p5: float, signs: tuple[int, int, int, int]):
        s2, s3, s4, s5 = signs
        a2 = s2 * p2 * RAD_PER_STEP
        a3 = a2 + s3 * p3 * RAD_PER_STEP
        a4 = a3 + s4 * p4 * RAD_PER_STEP
        a5 = a4 + s5 * p5 * RAD_PER_STEP
        return a2, a3, a4, a5

    def predict_loads(self, p2: float, p3: float, p4: float, p5: float) -> dict[str, float]:
        """由原始编码器位置 (0-4095) 预测四个关节的重力负载（带符号），返回 {joint: load}。"""
        a = self._abs_angles(p2, p3, p4, p5, self.signs)
        loads: dict[str, float] = {}
        for k, joint in enumerate(PITCH_JOINTS):
            c = self.coeffs[joint]
            t = self.intercepts.get(joint, 0.0)
            for m in range(len(c) // 2):
                ang = a[k + m]
                t += c[2 * m] * math.sin(ang) + c[2 * m + 1] * math.cos(ang)
            loads[joint] = t
        return loads

    def torque_limits(self, p2: float, p3: float, p4: float, p5: float, margin: float | dict[str, float]) -> dict[str, int]:
        """返回四个关节应写的 Torque_Limit（0-1000，已含 margin）。

        margin 可为标量（所有关节同一值）或 {joint: 值}（按关节分开——负载/建模精度不同的关节用不同 margin）。
        """
        loads = self.predict_loads(p2, p3, p4, p5)
        if isinstance(margin, dict):
            return {
                j: int(round(min(MAX_TORQUE_LIMIT, abs(loads[j]) + margin.get(j, 0.0))))
                for j in PITCH_JOINTS
            }
        return {
            j: int(round(min(MAX_TORQUE_LIMIT, abs(loads[j]) + margin)))
            for j in PITCH_JOINTS
        }

    # ------------------------------------------------------------------
    # 序列化
    # ------------------------------------------------------------------
    def to_dict(self) -> dict:
        return {
            "signs": list(self.signs),
            "coeffs": self.coeffs,
            "intercepts": self.intercepts,
            "rmse": self.rmse,
            "n_samples": self.n_samples,
            "joint_rmse": self.joint_rmse,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "GravityModel":
        return cls(
            signs=tuple(d["signs"]),
            coeffs={k: list(v) for k, v in d["coeffs"].items()},
            intercepts={k: float(v) for k, v in (d.get("intercepts") or {}).items()},
            rmse=float(d.get("rmse", 0.0)),
            n_samples=int(d.get("n_samples", 0)),
            joint_rmse={k: float(v) for k, v in (d.get("joint_rmse") or {}).items()},
        )

    def save(self, path: str | Path) -> None:
        Path(path).write_text(json.dumps(self.to_dict(), indent=2), encoding="utf-8")

    @classmethod
    def load(cls, path: str | Path) -> "GravityModel":
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))


def _fit_sign(samples: list[tuple[float, ...]], signs: tuple[int, int, int, int]):
    """给定符号组合，逐关节独立拟合，返回 (coeffs, 总 rmse, 各关节 rmse)。"""
    p2 = np.array([s[0] for s in samples])
    p3 = np.array([s[1] for s in samples])
    p4 = np.array([s[2] for s in samples])
    p5 = np.array([s[3] for s in samples])
    n = len(samples)

    s2, s3, s4, s5 = signs
    a2 = s2 * p2 * RAD_PER_STEP
    a3 = a2 + s3 * p3 * RAD_PER_STEP
    a4 = a3 + s4 * p4 * RAD_PER_STEP
    a5 = a4 + s5 * p5 * RAD_PER_STEP
    angles = [a2, a3, a4, a5]

    coeffs: dict[str, list[float]] = {}
    intercepts: dict[str, float] = {}
    joint_rmse: dict[str, float] = {}
    total_sq = 0.0
    for k, joint in enumerate(PITCH_JOINTS):
        y = np.array([s[4 + k] for s in samples])
        cols = []
        for m in range(k, k + JOINT_NUM_ANGLES[joint]):  # 只取本关节需要的下游绝对角
            cols.append(np.sin(angles[m]))
            cols.append(np.cos(angles[m]))
        cols.append(np.ones(n))  # 常数项：sin/cos 均值为 0，无法表示摩擦/装配的 DC 偏移
        X = np.column_stack(cols)
        c, *_ = np.linalg.lstsq(X, y, rcond=None)
        resid = y - X @ c
        coeffs[joint] = c[:-1].tolist()  # 前 2*num 个是 sin/cos，最后一个单独存为常数项
        intercepts[joint] = float(c[-1])
        joint_rmse[joint] = float(np.sqrt(np.mean(resid ** 2)))
        total_sq += float(np.sum(resid ** 2))

    rmse = float(np.sqrt(total_sq / (4 * n)))
    return coeffs, intercepts, rmse, joint_rmse


def fit_gravity_model(samples: list[tuple[float, ...]]) -> GravityModel:
    """
    由标定样本拟合重力模型。

    每个样本是一个 8 元组：
        (p2, p3, p4, p5, load2, load3, load4, load5)
    其中 p* 是原始编码器位置 (0-4095)，load* 是对应关节的 Present_Load（带符号）。
    """
    if len(samples) < 8:
        raise ValueError(f"样本太少（{len(samples)}），至少需要 8 个不同位姿（建议 15 个左右）。")

    best = None
    for s2 in (1, -1):
        for s3 in (1, -1):
            for s4 in (1, -1):
                for s5 in (1, -1):
                    coeffs, intercepts, rmse, joint_rmse = _fit_sign(samples, (s2, s3, s4, s5))
                    if best is None or rmse < best["rmse"]:
                        best = {"rmse": rmse, "signs": (s2, s3, s4, s5), "coeffs": coeffs,
                                "intercepts": intercepts, "joint_rmse": joint_rmse}

    return GravityModel(
        signs=best["signs"],
        coeffs=best["coeffs"],
        intercepts=best["intercepts"],
        rmse=best["rmse"],
        n_samples=len(samples),
        joint_rmse=best["joint_rmse"],
    )


class GravityCompensator:
    """运行时重力补偿器：包一个 FeetechMotorsBus，每 step 读位置 -> 算力矩 -> 写 Goal / Torque_Limit。

    目标角更新策略（按优先级，stiffness 优先）：
      - stiffness>0（默认，推荐）：Goal = 当前位置 − 重力负载/stiffness。舵机 P 项输出
        恰好 = 重力负载 的恒定托举力，方向随位姿跟踪、无固定锚点，
        因此松手停在哪、不下坠、不回原位、动起来轻。stiffness 即舵机 P 增益
        （load/编码器步，默认 32；可按关节指定，下垂 -> 减小，上浮 -> 增大）。
      - track_goal=True：Goal=当前位置（无偏移），I 项弱时会持续下坠，不推荐。
      - goal_threshold>0：位置偏离目标超阈值才更新 Goal（起步有卡顿）。
      - 三者都关：目标永久固定（稳定，但会回原位）。
    """

    def __init__(self, bus, model: GravityModel, margin: float = 20.0,
                 stiffness: float | dict[str, float] = 32.0,
                 track_goal: bool = False, goal_threshold: float = 0.0,
                 anti_droop: bool = False, anti_droop_gain: float = 1.0,
                 anti_droop_kp: float = 30.0, anti_droop_threshold: float = 5.0,
                 anti_droop_max: float = 400.0, corr_bins: int = 64):
        self.bus = bus
        self.model = model
        self.margin = margin
        self.stiffness = stiffness
        if isinstance(stiffness, dict) and any(
            j not in stiffness or not math.isfinite(stiffness[j]) or stiffness[j] <= 0 for j in PITCH_JOINTS
        ):
            raise ValueError("Per-joint gravity stiffness must be positive and finite for every pitch joint")
        self.track_goal = track_goal
        self.goal_threshold = goal_threshold
        self.anti_droop = anti_droop
        self.anti_droop_gain = anti_droop_gain
        self.anti_droop_kp = anti_droop_kp
        self.anti_droop_threshold = anti_droop_threshold
        self.anti_droop_max = anti_droop_max
        self.corr_bins = corr_bins
        self._goal = None  # 阈值模式下当前伺服目标角 {joint: int}
        # 抗下垂：按"绝对角"分格的校正表(替代标量 bias)。每格各自学该角度下的模型误差，
        # 换位姿时查表即时生效，不再有标量的"旧值错用"与 2~3 秒积分滞后。
        self._corr = {j: [0.0] * corr_bins for j in PITCH_JOINTS}
        self._visited = {j: [False] * corr_bins for j in PITCH_JOINTS}  # 各绝对角格是否被走过(诊断)
        self._last_corr = {j: 0.0 for j in PITCH_JOINTS}  # 当前查表得到的修正量(供诊断打印)
        self._prev_pos = None  # 上一步位置 {joint: int}
        self._vel_ema = {j: 0.0 for j in PITCH_JOINTS}  # 平滑速度 {joint: steps/step}

    def step(self) -> dict[str, int]:
        joints = list(PITCH_JOINTS)
        pos = self.bus.sync_read("Present_Position", joints, normalize=False)
        p2 = pos["heavy_joint"]
        p3 = pos["shoulder_lift"]
        p4 = pos["elbow_flex"]
        p5 = pos["wrist_flex"]

        loads = self.model.predict_loads(p2, p3, p4, p5)

        # 抗下垂：软件闭环，只在偏移模式下生效（fixed-goal 等模式走舵机自身闭环）。
        # 修正量不再是标量 bias，而是按"绝对角"查表得到的该角度专属修正量。
        if self.anti_droop and (isinstance(self.stiffness, dict) or self.stiffness > 0):
            angles = dict(zip(PITCH_JOINTS, self.model._abs_angles(p2, p3, p4, p5, self.model.signs)))
            self._update_anti_droop(pos, angles)
            for j in joints:
                v = self._vel_ema[j]
                # P 项(速度阻尼)即时摁住下坠速度；查表项(校正)消稳态误差。死区外(=人手掰动)冻结
                p_corr = self.anti_droop_kp * v if abs(v) < self.anti_droop_threshold else 0.0
                c = self._corr_lookup(j, angles[j])
                self._last_corr[j] = c
                loads[j] += c + p_corr

        # Torque_Limit 按"含修正后的负载"计算，保证舵机有足够上限输出修正后的托力
        if isinstance(self.margin, dict):
            limits = {
                j: int(round(min(MAX_TORQUE_LIMIT, abs(loads[j]) + self.margin.get(j, 0.0))))
                for j in joints
            }
        else:
            limits = {
                j: int(round(min(MAX_TORQUE_LIMIT, abs(loads[j]) + self.margin)))
                for j in joints
            }

        if isinstance(self.stiffness, dict) or self.stiffness > 0:
            # 偏移模式：Goal = 当前位置 − 负载/K，让 P 项输出恒定托举力（随位姿跟踪、无锚点）。
            # 注意是减号：本舵机方向反相(load 为负=托举时 goal 需在上方)，+ 会往低处驱动(实测主动下坠)。
            goal = {}
            for j in joints:
                stiffness = self.stiffness[j] if isinstance(self.stiffness, dict) else self.stiffness
                g = int(round(pos[j] - loads[j] / stiffness))
                goal[j] = max(0, min(4095, g))
            self.bus.sync_write("Goal_Position", goal, normalize=False)
        elif self.track_goal:
            self.bus.sync_write("Goal_Position", {j: int(pos[j]) for j in joints}, normalize=False)
        elif self.goal_threshold > 0:
            if self._goal is None:
                self._goal = {j: int(pos[j]) for j in joints}
                self.bus.sync_write("Goal_Position", self._goal, normalize=False)
            else:
                update = {}
                for j in joints:
                    if abs(pos[j] - self._goal[j]) >= self.goal_threshold:
                        update[j] = int(pos[j])
                        self._goal[j] = int(pos[j])
                if update:
                    self.bus.sync_write("Goal_Position", update, normalize=False)

        self.bus.sync_write("Torque_Limit", limits, normalize=False)
        return limits

    @property
    def corr(self) -> dict[str, float]:
        """当前查表得到的抗下垂修正量（load 量纲，仅供诊断打印）。"""
        return dict(self._last_corr)

    @property
    def corr_filled(self) -> dict[str, int]:
        """各关节校正表里已学到的格子数（便于观察学习进度）。"""
        return {j: sum(1 for x in self._corr[j] if x != 0.0) for j in PITCH_JOINTS}

    @property
    def corr_visited(self) -> dict[str, int]:
        """各关节绝对角实际扫过的格子数（受关节行程限制，通常 < corr_bins=64）。"""
        return {j: sum(1 for x in self._visited[j] if x) for j in PITCH_JOINTS}

    def _update_anti_droop(self, pos: dict[str, int], angles: dict[str, float]) -> None:
        """软件抗下垂积分器（偏移模式下用，替代舵机内部 I 项缺乏抗饱和的积分）。

        读位置速度：缓慢漂移(=下垂)时积累修正量去抵消；快速移动(=人手掰)时冻结，
        避免跟人手较劲。修正量不是标量，而是按"绝对角"分格存进校正表：每个角度格
        各自收敛到该角度下的模型误差，从而换位姿时查表即时生效、不再有旧值错用。
        """
        if self._prev_pos is None:
            self._prev_pos = dict(pos)
            return
        for j in PITCH_JOINTS:
            idx, _ = self._corr_bin(angles[j])
            self._visited[j][idx] = True
            dv = pos[j] - self._prev_pos[j]
            # 处理 0/4095 编码器回绕
            if dv > 2048:
                dv -= 4096
            elif dv < -2048:
                dv += 4096
            # 平滑速度（EMA），压掉亚步进量化噪声
            self._vel_ema[j] = 0.85 * self._vel_ema[j] + 0.15 * dv
            v = self._vel_ema[j]
            # 死区：只有"缓慢漂移"才当成下垂纠正；快速移动冻结
            if abs(v) < self.anti_droop_threshold:
                # 速度 v ∝ (Δ − corr)，Δ=模型误差；corr += gain·v 收敛到 Δ。
                # 本臂各关节 model 为负(托举)，下垂时 v<0，需 corr 更负增强托力，
                # += 正好让 v<0 时 corr 下降。-= 是正反馈会发散。
                self._corr_update(j, angles[j], self.anti_droop_gain * v)
        self._prev_pos = dict(pos)

    def _corr_bin(self, angle: float) -> tuple[int, float]:
        """绝对角(弧度) -> (格子下标, 小数部分)，用于校正表线性插值。"""
        f = (angle % (2 * math.pi)) / (2 * math.pi) * self.corr_bins
        idx = int(f) % self.corr_bins
        return idx, f - int(f)

    def _corr_lookup(self, j: str, angle: float) -> float:
        """查表：当前绝对角对应的校正量(相邻两格线性插值)。"""
        idx, frac = self._corr_bin(angle)
        nxt = (idx + 1) % self.corr_bins
        return (1.0 - frac) * self._corr[j][idx] + frac * self._corr[j][nxt]

    def _corr_update(self, j: str, angle: float, delta: float) -> None:
        """把增量 delta 按线性插值写进当前角度的两个相邻格，并做限幅。"""
        idx, frac = self._corr_bin(angle)
        nxt = (idx + 1) % self.corr_bins
        self._corr[j][idx] += (1.0 - frac) * delta
        self._corr[j][nxt] += frac * delta
        m = self.anti_droop_max
        for k in (idx, nxt):
            if self._corr[j][k] > m:
                self._corr[j][k] = m
            elif self._corr[j][k] < -m:
                self._corr[j][k] = -m
