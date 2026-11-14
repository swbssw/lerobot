"""Initialization sequence and keepalive checks with no real robot connection."""

import unittest
from unittest.mock import Mock, patch

from examples.jaka import initialize as init_module
from examples.jaka import power_on as power_module
from examples.jaka.reset_to_reference import REFERENCE_DEG


class InitializationTest(unittest.TestCase):
    def test_preview_calls_reset_preview_without_powering(self):
        sdk, config = Mock(), Mock()
        with (
            patch.object(init_module, "power_on_and_enable") as power,
            patch.object(init_module, "reset_to_reference") as reset,
        ):
            init_module.initialize(sdk, REFERENCE_DEG, config)
        power.assert_not_called()
        self.assertFalse(reset.call_args.kwargs["execute"])
        self.assertIs(reset.call_args.kwargs["gripper_config"], config)

    def test_execute_powers_before_reset(self):
        events = []
        with (
            patch.object(init_module, "power_on_and_enable", side_effect=lambda *a: events.append("power")),
            patch.object(
                init_module, "reset_to_reference", side_effect=lambda *a, **k: events.append("reset")
            ),
        ):
            init_module.initialize(Mock(), REFERENCE_DEG, Mock(), execute=True)
        self.assertEqual(events, ["power", "reset"])

    def test_power_failure_blocks_reset(self):
        with (
            patch.object(init_module, "power_on_and_enable", side_effect=RuntimeError("not enabled")),
            patch.object(init_module, "reset_to_reference") as reset,
        ):
            with self.assertRaises(RuntimeError):
                init_module.initialize(Mock(), REFERENCE_DEG, Mock(), execute=True)
        reset.assert_not_called()

    def test_invalid_target_never_powers(self):
        with patch.object(init_module, "power_on_and_enable") as power:
            with self.assertRaises(ValueError):
                init_module.initialize(Mock(), [float("nan")] * 6, Mock(), execute=True)
        power.assert_not_called()

    def test_estop_blocks_power_and_enable(self):
        sdk = Mock()
        sdk.get_robot_state.return_value = (0, [1, 0, 0])
        with self.assertRaises(RuntimeError):
            power_module.power_on_and_enable(sdk, 0)
        sdk.power_on.assert_not_called()
        sdk.enable_robot.assert_not_called()

    def test_power_and_enable_verify_actual_states(self):
        sdk = Mock()
        sdk.get_robot_state.side_effect = [(0, [0, 0, 0]), (0, [0, 1, 0]), (0, [0, 1, 1])]
        sdk.power_on.return_value = (0,)
        sdk.enable_robot.return_value = (0,)
        power_module.power_on_and_enable(sdk, 0)
        sdk.power_on.assert_called_once()
        sdk.enable_robot.assert_called_once()
        self.assertEqual(sdk.get_robot_state.call_count, 3)

    def test_failed_power_feedback_blocks_enable(self):
        sdk = Mock()
        sdk.get_robot_state.return_value = (0, [0, 0, 0])
        sdk.power_on.return_value = (0,)
        with self.assertRaises(RuntimeError):
            power_module.power_on_and_enable(sdk, 0)
        sdk.enable_robot.assert_not_called()

    def test_hold_streams_measured_position_then_disables_on_interrupt(self):
        gripper = Mock()
        gripper.read_position_deg.return_value = 0.12
        with (
            patch.object(init_module, "DamiaoGripper", return_value=gripper),
            patch.object(init_module.time, "sleep", side_effect=[None, KeyboardInterrupt]),
        ):
            with self.assertRaises(KeyboardInterrupt):
                init_module.hold_gripper(Mock())
        self.assertEqual(gripper.move_to_deg.call_count, 2)
        gripper.move_to_deg.assert_called_with(0.12)
        gripper.disconnect.assert_called_once()

    def test_hold_fault_disables_without_reconnect(self):
        gripper = Mock()
        gripper.read_position_deg.return_value = 0.0
        gripper.move_to_deg.side_effect = RuntimeError("fault D")
        with patch.object(init_module, "DamiaoGripper", return_value=gripper):
            with self.assertRaises(RuntimeError):
                init_module.hold_gripper(Mock())
        gripper.connect.assert_called_once()
        gripper.disconnect.assert_called_once()

    def test_cli_hold_only_after_successful_execution(self):
        for execute, exit_after, fail in ((False, False, False), (True, False, False),
                                          (True, True, False), (True, False, True)):
            with self.subTest(execute=execute, exit_after=exit_after, fail=fail):
                argv = ["initialize.py", "--ip=10.5.5.100"]
                if execute:
                    argv.append("--execute")
                if exit_after:
                    argv.append("--exit-after-reset")
                with (
                    patch("sys.argv", argv),
                    patch.object(init_module._system, "choose", return_value={
                        "sdk_python": "unused", "sdk_dir": "unused"
                    }),
                    patch.object(init_module, "JakaSDKBridge") as bridge,
                    patch.object(init_module, "initialize") as initialize,
                    patch.object(init_module, "hold_gripper") as hold,
                ):
                    bridge.return_value.__enter__.return_value.login.return_value = (0,)
                    if fail:
                        initialize.side_effect = RuntimeError("reset failed")
                        with self.assertRaises(RuntimeError):
                            init_module.main()
                    else:
                        init_module.main()
                    self.assertEqual(hold.call_count, int(execute and not exit_after and not fail))
                    self.assertEqual(initialize.call_args.kwargs["execute"], execute)


if __name__ == "__main__":
    unittest.main()
