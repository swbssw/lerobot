"""Hardware-free checks for gripper/JAKA reset ordering and failure handling."""

import math
import unittest
from unittest.mock import Mock, patch

from examples.jaka import reset_to_reference as reset_module
from lerobot.robots.jaka_zu7_follower.config_jaka_zu7_follower import JAKAZu7FollowerConfig


class CombinedResetTest(unittest.TestCase):
    def setUp(self) -> None:
        self.config = JAKAZu7FollowerConfig(
            ip_address="unused",
            gripper_can_channel="unused",
            gripper_can_interface="dm_usb2can",
            gripper_can_id=7,
            gripper_feedback_id=20,
            gripper_open_deg=0,
            gripper_closed_deg=57.2958,
            gripper_control_mode=4,
            gripper_current_limit_pct=2.5,
        )
        self.events: list[str] = []
        self.sdk = Mock(closed=False)
        self.sdk.get_robot_state.return_value = (0, [0, 1, 1])
        self.sdk.get_joint_position.return_value = (0, [0.0] * 6)
        self.sdk.is_in_pos.return_value = (0, True)
        self.sdk.is_in_servomove.return_value = (0, False)
        self.sdk.get_tool_id.return_value = (0, 0)
        self.sdk.motion_abort.return_value = (0,)

        def move(*args: object) -> tuple[int]:
            self.events.append("arm")
            self.sdk.get_joint_position.return_value = (
                0, [math.radians(angle) for angle in reset_module.REFERENCE_DEG]
            )
            return (0,)

        self.sdk.joint_move_extend.side_effect = move

    def test_preview_never_enables_or_moves(self) -> None:
        with patch.object(reset_module, "reset_gripper") as gripper:
            reset_module.reset_to_reference(
                self.sdk, reset_module.REFERENCE_DEG, gripper_config=self.config
            )
        gripper.assert_called_once_with(self.config, 0, execute=False)
        self.sdk.joint_move_extend.assert_not_called()
        self.sdk.servo_move_enable.assert_not_called()

    def test_gripper_completes_before_arm_move(self) -> None:
        with patch.object(reset_module, "reset_gripper", side_effect=lambda *a, **kw: self.events.append("jaw")):
            reset_module.reset_to_reference(
                self.sdk, reset_module.REFERENCE_DEG, execute=True, gripper_config=self.config
            )
        self.assertEqual(self.events, ["jaw", "arm"])
        self.sdk.motion_abort.assert_called_once()

    def test_gripper_timeout_blocks_arm_move(self) -> None:
        with patch.object(reset_module, "reset_gripper", side_effect=TimeoutError("jaw stuck")):
            with self.assertRaisesRegex(TimeoutError, "jaw stuck"):
                reset_module.reset_to_reference(
                    self.sdk, reset_module.REFERENCE_DEG, execute=True, gripper_config=self.config
                )
        self.sdk.joint_move_extend.assert_not_called()

    def test_already_reset_arm_still_resets_gripper(self) -> None:
        self.sdk.get_joint_position.return_value = (
            0, [math.radians(angle) for angle in reset_module.REFERENCE_DEG]
        )
        with patch.object(reset_module, "reset_gripper") as gripper:
            reset_module.reset_to_reference(
                self.sdk, reset_module.REFERENCE_DEG, execute=True, gripper_config=self.config
            )
        gripper.assert_called_once_with(self.config, 0, execute=True, timeout_s=15.0)
        self.sdk.joint_move_extend.assert_not_called()

    def test_unready_arm_blocks_gripper_and_arm(self) -> None:
        self.sdk.get_robot_state.return_value = (0, [1, 1, 1])
        with patch.object(reset_module, "reset_gripper") as gripper:
            with self.assertRaises(RuntimeError):
                reset_module.reset_to_reference(
                    self.sdk, reset_module.REFERENCE_DEG, execute=True, gripper_config=self.config
                )
        gripper.assert_not_called()
        self.sdk.joint_move_extend.assert_not_called()

    def test_cli_includes_gripper_by_default_and_allows_skip(self) -> None:
        for skip in (False, True):
            with self.subTest(skip=skip):
                argv = ["reset_to_reference.py", "--ip=10.5.5.100"]
                if skip:
                    argv.append("--skip-gripper")
                with (
                    patch("sys.argv", argv),
                    patch.object(reset_module._system, "choose", return_value={
                        "sdk_python": "unused", "sdk_dir": "unused"
                    }),
                    patch.object(reset_module, "JakaSDKBridge") as bridge,
                    patch.object(reset_module, "reset_to_reference") as reset,
                ):
                    bridge.return_value.__enter__.return_value = self.sdk
                    self.sdk.login.return_value = (0,)
                    reset_module.main()
                kwargs = reset.call_args.kwargs
                self.assertFalse(kwargs["execute"])
                if skip:
                    self.assertIsNone(kwargs["gripper_config"])
                else:
                    config = kwargs["gripper_config"]
                    self.assertEqual(config.gripper_open_deg, 0)
                    self.assertEqual(config.gripper_closed_deg, 120)
                    self.assertEqual(config.gripper_max_velocity_deg_s, 90)
                    self.assertEqual(config.gripper_control_mode, 4)
                    self.assertEqual(config.gripper_current_limit_pct, 1.61)


if __name__ == "__main__":
    unittest.main()
