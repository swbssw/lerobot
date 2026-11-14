import math
import struct
from types import SimpleNamespace

import pytest

from examples.jaka import reset_gripper as reset_module
from tests.robots.test_jaka_damiao_gripper import gripper_config


def test_preview_reads_motor_without_enable_or_motion(monkeypatch, capsys):
    class Bus:
        def __init__(self):
            self.sent = []
            self.replies = []
            self.closed = False

        def send(self, can_id, data):
            self.sent.append((can_id, data))
            if data[2] == 0x33:
                register = data[3]
                payload = struct.pack("<f", math.radians(5)) if register == 0x50 else struct.pack("<I", 2)
                reply = bytes((3, 0, 0x33, register)) + payload
            else:
                reply = b"\x03" + b"\0" * 7
            self.replies.append(SimpleNamespace(arbitration_id=0x13, data=reply))

        def recv(self, timeout):
            return self.replies.pop(0) if self.replies else None

        def shutdown(self):
            self.closed = True

    bus = Bus()
    monkeypatch.setattr(reset_module, "DamiaoUSB2CAN", lambda port: bus)
    reset_module.reset_gripper(gripper_config(gripper_can_interface="dm_usb2can"))
    assert "Preview only" in capsys.readouterr().out
    assert bus.closed
    assert [frame[0] for frame in bus.sent] == [0x7FF] * 3
    assert [frame[1][2] for frame in bus.sent] == [0x33, 0x33, 0xCC]


def test_execute_streams_until_measured_arrival_then_disables(monkeypatch):
    gripper = SimpleNamespace(
        connected=False, commands=[], readings=iter((10.0, 0.1, 0.1, 0.1)), disconnected=False
    )
    gripper.connect = lambda: setattr(gripper, "connected", True)
    gripper.move_to_deg = lambda angle: gripper.commands.append(angle)
    gripper.read_position_deg = lambda: next(gripper.readings)
    gripper.disconnect = lambda: setattr(gripper, "disconnected", True)
    monkeypatch.setattr(reset_module, "DamiaoGripper", lambda config: gripper)
    monkeypatch.setattr(reset_module.time, "sleep", lambda duration: None)
    reset_module.reset_gripper(
        gripper_config(gripper_can_interface="dm_usb2can"), execute=True
    )
    assert gripper.connected and gripper.disconnected
    assert gripper.commands == [0.0] * 4


def test_execute_disables_after_feedback_failure(monkeypatch):
    gripper = SimpleNamespace(disconnected=False)
    gripper.connect = lambda: None
    gripper.move_to_deg = lambda angle: None
    gripper.read_position_deg = lambda: (_ for _ in ()).throw(TimeoutError("feedback lost"))
    gripper.disconnect = lambda: setattr(gripper, "disconnected", True)
    monkeypatch.setattr(reset_module, "DamiaoGripper", lambda config: gripper)
    with pytest.raises(TimeoutError, match="feedback lost"):
        reset_module.reset_gripper(gripper_config(gripper_can_interface="dm_usb2can"), execute=True)
    assert gripper.disconnected
