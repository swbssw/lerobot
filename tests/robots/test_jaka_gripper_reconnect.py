"""Exercise startup communication-fault recovery without real CAN hardware."""

import math
import struct
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from lerobot.robots.jaka_zu7_follower import damiao_gripper as driver_module
from lerobot.robots.jaka_zu7_follower.config_jaka_zu7_follower import JAKAZu7FollowerConfig
from lerobot.robots.jaka_zu7_follower.damiao_gripper import DamiaoGripper


class MotorBus:
    def __init__(self, state: int = 0xD, clear_state: int = 0):
        self.state = state
        self.clear_state = clear_state
        self.sent = []
        self.replies = []
        self.closed = False
        self.position = 0.4

    def send(self, message):
        data = bytes(message.data)
        self.sent.append((message.arbitration_id, data))
        if message.arbitration_id == 0x7FF:
            register = data[3]
            values = {8: 7, 7: 20, 10: 4, 9: 10000}
            value = (
                struct.pack("<f", self.position) if register == 80 else struct.pack("<I", values[register])
            )
            reply = bytes((7, 0, 0x33, register)) + value
        else:
            if data == b"\xff" * 7 + b"\xfb":
                self.state = self.clear_state
                self.position = 0.5  # Re-read the position after recovery, before enabling.
            elif data == b"\xff" * 7 + b"\xfc":
                self.state = 1
            elif data == b"\xff" * 7 + b"\xfd":
                self.state = 0
            reply = bytes((self.state << 4 | 7,)) + b"\0" * 7
        self.replies.append(SimpleNamespace(arbitration_id=20, data=reply))

    def recv(self, timeout):
        return self.replies.pop(0) if self.replies else None

    def shutdown(self):
        self.closed = True


class GripperReconnectTest(unittest.TestCase):
    def setUp(self):
        self.config = JAKAZu7FollowerConfig(
            ip_address="unused", gripper_can_channel="unused", gripper_can_interface="socketcan",
            gripper_can_id=7, gripper_feedback_id=20, gripper_open_deg=0, gripper_closed_deg=90,
            gripper_control_mode=4, gripper_current_limit_pct=2.5,
        )

    def driver(self, bus):
        module = SimpleNamespace(
            interface=SimpleNamespace(Bus=lambda **kwargs: bus),
            Message=lambda **kwargs: SimpleNamespace(**kwargs),
        )
        patcher = patch.object(driver_module, "importlib", SimpleNamespace(import_module=lambda name: module))
        patcher.start()
        self.addCleanup(patcher.stop)
        return DamiaoGripper(self.config)

    def test_startup_d_clears_once_and_holds_fresh_position(self):
        bus = MotorBus()
        gripper = self.driver(bus)
        gripper.connect()
        clear = (0x307, b"\xff" * 7 + b"\xfb")
        enable = (0x307, b"\xff" * 7 + b"\xfc")
        self.assertEqual(bus.sent.count(clear), 1)
        self.assertLess(bus.sent.index(clear), bus.sent.index(enable))
        self.assertFalse(any(data == b"\xff" * 7 + b"\xfe" for _, data in bus.sent))
        self.assertAlmostEqual(gripper._start_motor_deg, math.degrees(0.5), places=4)
        self.assertAlmostEqual(struct.unpack("<fHH", bus.sent[-1][1])[0], 0.5)
        gripper.disconnect()
        self.assertTrue(bus.closed)

    def test_persistent_d_does_not_retry_or_enable(self):
        bus = MotorBus(clear_state=0xD)
        with self.assertRaises(RuntimeError):
            self.driver(bus).connect()
        self.assertEqual(sum(data[-1:] == b"\xfb" for _, data in bus.sent), 1)
        self.assertNotIn((0x307, b"\xff" * 7 + b"\xfc"), bus.sent)
        self.assertTrue(bus.closed)

    def test_other_faults_are_not_cleared(self):
        for state in (8, 9, 10, 11, 12, 14):
            with self.subTest(state=state):
                bus = MotorBus(state=state)
                with self.assertRaises(RuntimeError):
                    self.driver(bus).connect()
                self.assertNotIn((0x307, b"\xff" * 7 + b"\xfb"), bus.sent)
                self.assertNotIn((0x307, b"\xff" * 7 + b"\xfc"), bus.sent)

    def test_running_d_stops_without_clear_or_reenable(self):
        bus = MotorBus(state=0)
        gripper = self.driver(bus)
        gripper.connect()
        bus.sent.clear()
        bus.state = 0xD
        with self.assertRaises(RuntimeError):
            gripper.send(50)
        self.assertEqual(len(bus.sent), 1)
        gripper.disconnect()

    def test_normal_exit_then_reconnect_does_not_clear_or_change_zero(self):
        bus = MotorBus(state=0)
        gripper = self.driver(bus)
        gripper.connect()
        gripper.send(80)
        gripper.disconnect()
        bus.sent.clear()
        gripper.connect()
        self.assertEqual(bus.state, 1)
        self.assertIsNone(gripper._start_leader_pct)
        self.assertFalse(any(data == b"\xff" * 7 + b"\xfb" for _, data in bus.sent))
        self.assertAlmostEqual(struct.unpack("<fHH", bus.sent[-1][1])[0], 0.4)
        gripper.disconnect()


if __name__ == "__main__":
    unittest.main()
