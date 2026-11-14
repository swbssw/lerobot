import math
from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig

from ..config import RobotConfig


@RobotConfig.register_subclass("jaka_zu7_follower")
@dataclass
class JAKAZu7FollowerConfig(RobotConfig):
    """Configuration for a JAKA Zu 7 follower arm."""

    ip_address: str
    # Set both to run jkrc in a separate, SDK-compatible Python (e.g. Python 3.10).
    sdk_python: str | None = None
    sdk_dir: str | None = None
    sdk_timeout_s: float = 5.0
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
    auto_power_on: bool = False
    auto_enable_robot: bool = False
    power_on_delay_s: float = 8.0
    servo_step_num: int = 2
    servo_api: str = "servo_j_extend"
    configure_servo_nlf: bool = False
    max_joint_jerk_deg_s3: float = 240.0
    relative_start: bool = False
    max_joint_velocity_deg_s: float = 15.0
    max_joint_acceleration_deg_s2: float = 30.0
    joint_signs: tuple[int, ...] = (1, 1, 1, 1, 1, 1)
    joint_offsets_deg: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    gripper_can_channel: str | None = None
    gripper_can_interface: str | None = None
    gripper_can_id: int | None = None
    gripper_feedback_id: int | None = None
    gripper_open_deg: float | None = None
    gripper_closed_deg: float | None = None
    gripper_max_velocity_deg_s: float = 90.0
    # -1: decreasing leader percentage closes the jaw; +1: increasing closes it.
    gripper_leader_sign: int = 1
    gripper_control_mode: int = 2
    # Mode 4 limits phase current as a percentage of the driver's maximum
    # phase current, not jaw force in N or a percentage of rated motor torque.
    gripper_current_limit_pct: float | None = None

    def __post_init__(self) -> None:
        super().__post_init__()
        if self.sdk_dir is not None and not self.sdk_python:
            raise ValueError("sdk_dir requires sdk_python to enable the SDK subprocess")
        if not math.isfinite(self.sdk_timeout_s) or self.sdk_timeout_s <= 0:
            raise ValueError("sdk_timeout_s must be finite and positive")
        if self.servo_step_num < 1:
            raise ValueError("servo_step_num must be at least 1")
        if self.servo_api not in {"servo_j", "servo_j_extend"}:
            raise ValueError("servo_api must be servo_j or servo_j_extend")
        if self.servo_api == "servo_j" and self.servo_step_num != 1:
            raise ValueError("servo_j uses a fixed 8 ms cycle; set servo_step_num=1")
        if not math.isfinite(self.max_joint_jerk_deg_s3) or self.max_joint_jerk_deg_s3 <= 0:
            raise ValueError("max_joint_jerk_deg_s3 must be finite and positive")
        if not math.isfinite(self.max_joint_velocity_deg_s) or self.max_joint_velocity_deg_s <= 0:
            raise ValueError("max_joint_velocity_deg_s must be positive")
        if not math.isfinite(self.max_joint_acceleration_deg_s2) or self.max_joint_acceleration_deg_s2 <= 0:
            raise ValueError("max_joint_acceleration_deg_s2 must be positive")
        if not math.isfinite(self.power_on_delay_s) or self.power_on_delay_s < 0:
            raise ValueError("power_on_delay_s must be non-negative")
        if len(self.joint_signs) != 6 or any(sign not in (-1, 1) for sign in self.joint_signs):
            raise ValueError("joint_signs must contain six values, each either -1 or 1")
        if len(self.joint_offsets_deg) != 6 or not all(
            math.isfinite(offset) for offset in self.joint_offsets_deg
        ):
            raise ValueError("joint_offsets_deg must contain six finite values")
        gripper_fields = (
            self.gripper_can_channel,
            self.gripper_can_interface,
            self.gripper_can_id,
            self.gripper_feedback_id,
            self.gripper_open_deg,
            self.gripper_closed_deg,
        )
        if any(value is not None for value in gripper_fields):
            if any(value is None for value in gripper_fields):
                raise ValueError(
                    "Gripper requires channel, interface, CAN ID, feedback ID, open and closed angles"
                )
            if self.gripper_can_interface not in ("socketcan", "slcan", "dm_usb2can"):
                raise ValueError("gripper_can_interface must be socketcan, slcan or dm_usb2can")
            if not self.gripper_can_channel:
                raise ValueError("gripper_can_channel must not be empty")
            if not isinstance(self.gripper_can_id, int) or not 1 <= self.gripper_can_id <= 15:
                raise ValueError("gripper_can_id must be an integer from 1 to 15")
            if not isinstance(self.gripper_feedback_id, int) or not 0 <= self.gripper_feedback_id <= 0x7FF:
                raise ValueError("gripper_feedback_id must be an 11-bit CAN ID")
            if not all(math.isfinite(value) for value in (self.gripper_open_deg, self.gripper_closed_deg)):
                raise ValueError("gripper open and closed angles must be finite")
            if self.gripper_open_deg == self.gripper_closed_deg:
                raise ValueError("gripper open and closed angles must differ")
        if not math.isfinite(self.gripper_max_velocity_deg_s) or self.gripper_max_velocity_deg_s <= 0:
            raise ValueError("gripper_max_velocity_deg_s must be finite and positive")
        if self.gripper_leader_sign not in (-1, 1):
            raise ValueError("gripper_leader_sign must be -1 or 1")
        if self.gripper_control_mode not in (2, 4):
            raise ValueError("gripper_control_mode must be 2 (position-velocity) or 4 (force-position)")
        if self.gripper_control_mode == 4:
            if self.gripper_can_channel is None:
                raise ValueError("Mode 4 requires complete gripper configuration")
            if self.gripper_current_limit_pct is None or not (
                math.isfinite(self.gripper_current_limit_pct) and 0.01 <= self.gripper_current_limit_pct <= 100
            ):
                raise ValueError("Mode 4 requires gripper_current_limit_pct between 0.01 and 100")
            if not 0.01 <= math.radians(self.gripper_max_velocity_deg_s) <= 100:
                raise ValueError("Mode 4 gripper speed must be between 0.01 and 100 rad/s")
        elif self.gripper_current_limit_pct is not None:
            raise ValueError("gripper_current_limit_pct requires gripper_control_mode=4")
