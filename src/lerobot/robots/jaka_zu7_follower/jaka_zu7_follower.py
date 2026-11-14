import importlib
import logging
import math
import time
from functools import cached_property
from typing import Any

from lerobot.cameras import DepthCamera, make_cameras_from_configs
from lerobot.lerobot_types import RobotAction, RobotObservation
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..robot import Robot
from .config_jaka_zu7_follower import JAKAZu7FollowerConfig
from .damiao_gripper import DamiaoGripper
from .sdk_bridge import JakaSDKBridge

logger = logging.getLogger(__name__)

# Keep these keys identical to the six arm joints in this project's SO-ARM101 leader.
JAKA_ZU7_JOINTS = (
    "shoulder_pan",
    "heavy_joint",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)
JAKA_SERVO_PERIOD_S = 0.008


def _limit_joint_step(
    position: list[float],
    velocity: list[float],
    target: list[float],
    dt: float,
    max_velocity: float,
    max_acceleration: float,
) -> tuple[list[float], list[float]]:
    next_position = []
    next_velocity = []

    for pos, vel, goal in zip(position, velocity, target, strict=True):
        error = goal - pos
        braking_speed = math.sqrt(2 * max_acceleration * abs(error))
        desired_velocity = math.copysign(min(max_velocity, braking_speed), error) if error else 0.0
        velocity_delta = max(-max_acceleration * dt, min(max_acceleration * dt, desired_velocity - vel))
        new_velocity = vel + velocity_delta
        new_position = pos + new_velocity * dt

        if error and (goal - new_position) * error <= 0:
            new_position = goal
            new_velocity = 0.0

        next_position.append(new_position)
        next_velocity.append(new_velocity)

    return next_position, next_velocity


class JAKAZu7Follower(Robot):
    """LeRobot follower driver for the six-axis JAKA Zu 7."""

    config_class = JAKAZu7FollowerConfig
    name = "jaka_zu7_follower"

    def __init__(self, config: JAKAZu7FollowerConfig):
        super().__init__(config)
        self.config = config
        self._sdk: Any | None = None
        self._logged_in = False
        self._servo_mode_enabled = False
        self._connected = False
        self._command_position_deg: list[float] | None = None
        self._command_velocity_deg_s = [0.0] * len(JAKA_ZU7_JOINTS)
        self._last_command_time: float | None = None
        self._leader_start_deg: list[float] | None = None
        self._follower_start_deg: list[float] | None = None
        self._gripper = DamiaoGripper(config) if config.gripper_can_channel is not None else None
        self.cameras = make_cameras_from_configs(config.cameras)

    @cached_property
    def observation_features(self) -> dict[str, type | tuple[int | None, ...]]:
        features: dict[str, type | tuple[int | None, ...]] = dict.fromkeys(self.action_features, float)
        for camera_name, camera in self.cameras.items():
            if getattr(camera, "use_rgb", True):
                features[camera_name] = (camera.height, camera.width, 3)
            if isinstance(camera, DepthCamera) and camera.use_depth:
                features[f"{camera_name}_depth"] = (camera.height, camera.width, 1)
        return features

    @cached_property
    def action_features(self) -> dict[str, type]:
        features = {f"{joint}.pos": float for joint in JAKA_ZU7_JOINTS}
        if self._gripper is not None:
            features["gripper.pos"] = float
        return features

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        return None

    def configure(self) -> None:
        return None

    @check_if_already_connected
    def connect(self, calibrate: bool = False) -> None:
        try:
            if self.config.sdk_python:
                self._sdk = JakaSDKBridge(
                    self.config.ip_address,
                    self.config.sdk_python,
                    sdk_dir=self.config.sdk_dir,
                    timeout_s=self.config.sdk_timeout_s,
                )
            else:
                try:
                    sdk_module = importlib.import_module("jkrc")
                except (ImportError, SystemError) as exc:
                    raise ImportError(
                        "Could not load the JAKA Python SDK module 'jkrc'. Set sdk_python and sdk_dir to "
                        "run the SDK in a compatible Python subprocess, or add the SDK directory to "
                        "PYTHONPATH and LD_LIBRARY_PATH for an SDK compatible with this Python."
                    ) from exc
                self._sdk = sdk_module.RC(self.config.ip_address)
            logger.info("JAKA SDK bridge ready, sending login...")
            self._check_sdk_result(self._sdk.login(), "login")
            self._logged_in = True
            logger.info("JAKA login ok")

            if self.config.auto_power_on:
                self._check_sdk_result(self._sdk.power_on(), "power_on")
                time.sleep(self.config.power_on_delay_s)
            if self.config.auto_enable_robot:
                self._check_sdk_result(self._sdk.enable_robot(), "enable_robot")

            self._command_position_deg = self._read_joints_deg()
            self._follower_start_deg = list(self._command_position_deg)
            self._leader_start_deg = None
            logger.info("JAKA joints read: %s", [round(v, 2) for v in self._command_position_deg])
            if self.config.configure_servo_nlf:
                self._check_sdk_result(
                    self._sdk.servo_move_use_joint_NLF(
                        self.config.max_joint_velocity_deg_s,
                        self.config.max_joint_acceleration_deg_s2,
                        self.config.max_joint_jerk_deg_s3,
                    ),
                    "servo_move_use_joint_NLF",
                )
            else:
                # Explicitly clear any filter left on the controller so disabling NLF
                # really takes effect instead of silently keeping the previous filter.
                self._check_sdk_result(
                    self._sdk.servo_move_use_none_filter(),
                    "servo_move_use_none_filter",
                )
            logger.info("JAKA servo filter configured, enabling servo...")
            self._check_sdk_result(self._sdk.servo_move_enable(True), "servo_move_enable(True)")
            self._servo_mode_enabled = True
            logger.info("JAKA servo enabled")
            if self._gripper is not None:
                self._gripper.connect()
            self._last_command_time = time.monotonic()
            self._connected = True

            for camera in self.cameras.values():
                camera.connect()

            logger.info("%s connected to JAKA controller at %s", self, self.config.ip_address)
        except Exception:
            try:
                self.disconnect()
            except Exception:
                logger.exception("Cleanup failed after JAKA connection error")
            raise

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        joints_deg = self._read_joints_deg()
        observation: RobotObservation = {
            f"{joint}.pos": position for joint, position in zip(JAKA_ZU7_JOINTS, joints_deg, strict=True)
        }
        if self._gripper is not None:
            observation["gripper.pos"] = self._gripper.read()

        for camera_name, camera in self.cameras.items():
            if getattr(camera, "use_rgb", True):
                observation[camera_name] = camera.read_latest()
            if isinstance(camera, DepthCamera) and camera.use_depth:
                observation[f"{camera_name}_depth"] = camera.read_latest_depth()

        return observation

    def get_joint_monitor(self) -> dict[str, Any]:
        """Read per-joint instantaneous current / torque / temperature / velocity.

        Parses get_robot_status() robot_monitor_data. Returns lists (length 6) of
        'current_a' (A), 'torque_nm' (Nm), 'temperature_c' (°C), 'velocity_rad_s' (rad/s),
        plus 'robot_avg_current_a' (A) and 'cab_temp_c' (°C). Non-finite values are None.
        """
        sdk = self._require_sdk()
        result = sdk.get_robot_status()
        self._check_sdk_result(result, "get_robot_status")
        status = result[1]
        # status is the RobotStatus struct (nested lists/tuples): [20] = robot_monitor_data =
        # (scbMajor, scbMinor, cabTemp, avgPower, avgCurrent, jointMonitorData[6]);
        # each joint monitor = (instCurrent, instVoltage, instTemperature, instVel, instTorq).
        monitor = status[20]
        joint_monitor = monitor[5]
        return {
            "current_a": [joint_monitor[j][0] for j in range(6)],
            "torque_nm": [joint_monitor[j][4] for j in range(6)],
            "temperature_c": [joint_monitor[j][2] for j in range(6)],
            "velocity_rad_s": [joint_monitor[j][3] for j in range(6)],
            "robot_avg_current_a": monitor[4],
            "cab_temp_c": monitor[2],
        }

    def get_joint_positions_deg(self) -> list[float]:
        """Read the six joint angles (degrees) from the controller encoder.

        Returned in the leader/observation space (joint_signs and joint_offsets_deg
        removed), matching get_observation(). Blocks for one SDK round-trip, so call
        it at a low rate (e.g. every second), not inside the 8 ms servo cadence.
        """
        return self._read_joints_deg()

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        allowed = set(self.action_features)
        extra = set(action) - allowed - {"gripper.pos"}
        if extra:
            raise KeyError(f"Unknown JAKA Zu 7 action keys: {sorted(extra)}")

        missing = allowed - action.keys()
        if missing:
            raise KeyError(f"Missing JAKA Zu 7 action keys: {sorted(missing)}")

        if self._gripper is not None:
            gripper_value = float(action["gripper.pos"])
            if not math.isfinite(gripper_value) or not 0 <= gripper_value <= 100:
                raise ValueError("SO-101 gripper action must be between 0 and 100 percent")

        target_deg = [float(action[f"{joint}.pos"]) for joint in JAKA_ZU7_JOINTS]
        if not all(math.isfinite(value) for value in target_deg):
            raise ValueError("JAKA joint targets must all be finite numbers")

        if self._command_position_deg is None:
            raise RuntimeError("JAKA follower has no initial joint position")
        if self.config.relative_start:
            if self._leader_start_deg is None:
                self._leader_start_deg = list(target_deg)
            if self._follower_start_deg is None:
                raise RuntimeError("JAKA follower has no starting joint position")
            target_deg = [
                follower + leader - start
                for follower, leader, start in zip(
                    self._follower_start_deg, target_deg, self._leader_start_deg, strict=True
                )
            ]
        now = time.monotonic()
        servo_window_s = self.config.servo_step_num * JAKA_SERVO_PERIOD_S
        elapsed_s = servo_window_s if self._last_command_time is None else now - self._last_command_time
        dt = max(0.001, min(elapsed_s, servo_window_s))

        command_deg, command_velocity_deg_s = _limit_joint_step(
            position=self._command_position_deg,
            velocity=self._command_velocity_deg_s,
            target=target_deg,
            dt=dt,
            max_velocity=self.config.max_joint_velocity_deg_s,
            max_acceleration=self.config.max_joint_acceleration_deg_s2,
        )
        controller_command_deg = [
            sign * value + offset
            for sign, offset, value in zip(
                self.config.joint_signs, self.config.joint_offsets_deg, command_deg, strict=True
            )
        ]
        command_rad = [math.radians(value) for value in controller_command_deg]
        sdk = self._require_sdk()
        if self.config.servo_api == "servo_j":
            result = sdk.servo_j(command_rad, 0)
        else:
            result = sdk.servo_j_extend(command_rad, 0, self.config.servo_step_num)
        self._check_sdk_result(result, self.config.servo_api)
        self._command_position_deg = command_deg
        self._command_velocity_deg_s = command_velocity_deg_s
        self._last_command_time = now

        sent = {f"{joint}.pos": value for joint, value in zip(JAKA_ZU7_JOINTS, command_deg, strict=True)}
        if self._gripper is not None:
            sent["gripper.pos"] = self._gripper.send(gripper_value)
        return sent

    def disconnect(self) -> None:
        first_error: Exception | None = None
        if self._gripper is not None:
            try:
                self._gripper.disconnect()
            except Exception as exc:
                first_error = exc
        for camera in self.cameras.values():
            try:
                if camera.is_connected:
                    camera.disconnect()
            except Exception as exc:
                first_error = first_error or exc

        if self._sdk is not None:
            if isinstance(self._sdk, JakaSDKBridge) and self._sdk.closed:
                # An interrupted/failed request closes stdin and reaps the worker,
                # which attempts servo cleanup and logout on EOF. Do not issue
                # more SDK requests to its closed pipe during outer cleanup.
                self._servo_mode_enabled = False
                self._logged_in = False
            if self._servo_mode_enabled:
                try:
                    # The Windows SDK returns -61 (ERR_OPERATION_TIMEOUT) on every
                    # servo_move_enable(False) even though the servo actually exits;
                    # treat it as success so clean shutdown does not raise.
                    self._check_sdk_result(
                        self._sdk.servo_move_enable(False),
                        "servo_move_enable(False)",
                        allowed_nonzero_codes=(-61,),
                    )
                except Exception as exc:
                    first_error = first_error or exc
                self._servo_mode_enabled = False
            if self._logged_in:
                try:
                    self._check_sdk_result(self._sdk.logout(), "logout")
                except Exception as exc:
                    first_error = first_error or exc
                self._logged_in = False
            if isinstance(self._sdk, JakaSDKBridge):
                try:
                    self._sdk.close()
                except Exception as exc:
                    first_error = first_error or exc

        self._sdk = None
        self._connected = False
        self._command_position_deg = None
        self._leader_start_deg = None
        self._follower_start_deg = None
        self._last_command_time = None
        self._command_velocity_deg_s = [0.0] * len(JAKA_ZU7_JOINTS)
        if first_error is not None:
            raise RuntimeError(f"Error while disconnecting {self}") from first_error

    def _read_joints_deg(self) -> list[float]:
        result = self._require_sdk().get_joint_position()
        self._check_sdk_result(result, "get_joint_position")
        if len(result) < 2 or len(result[1]) != len(JAKA_ZU7_JOINTS):
            raise RuntimeError("JAKA get_joint_position returned an unexpected joint vector")
        joints_rad = [float(value) for value in result[1]]
        if not all(math.isfinite(value) for value in joints_rad):
            raise RuntimeError("JAKA get_joint_position returned non-finite joint values")
        joints_deg = [math.degrees(value) for value in joints_rad]
        return [
            (value - offset) / sign
            for sign, offset, value in zip(
                self.config.joint_signs, self.config.joint_offsets_deg, joints_deg, strict=True
            )
        ]

    def _require_sdk(self) -> Any:
        if self._sdk is None:
            raise RuntimeError("JAKA SDK is not connected")
        return self._sdk

    @staticmethod
    def _check_sdk_result(
        result: Any, operation: str, allowed_nonzero_codes: tuple[int, ...] = ()
    ) -> None:
        if not isinstance(result, tuple | list) or not result or result[0] != 0:
            code = result[0] if isinstance(result, tuple | list) and result else result
            if code not in allowed_nonzero_codes:
                raise RuntimeError(f"JAKA SDK call {operation} failed with return value {code!r}")
