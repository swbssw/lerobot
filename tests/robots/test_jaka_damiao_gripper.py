import math
import struct
from types import SimpleNamespace

import pytest
import serial

from examples.jaka.probe_gripper import read_status
from lerobot.robots.jaka_zu7_follower.config_jaka_zu7_follower import JAKAZu7FollowerConfig
from lerobot.robots.jaka_zu7_follower.damiao_gripper import DamiaoGripper
from lerobot.robots.jaka_zu7_follower.damiao_usb2can import DamiaoUSB2CAN
from lerobot.robots.jaka_zu7_follower.jaka_zu7_follower import JAKAZu7Follower


def gripper_config(**changes):
    values = {
        "ip_address": "10.5.5.100",
        "gripper_can_channel": "can0",
        "gripper_can_interface": "socketcan",
        "gripper_can_id": 3,
        "gripper_feedback_id": 0x13,
        "gripper_open_deg": 0.0,
        "gripper_closed_deg": 90.0,
    }
    values.update(changes)
    return JAKAZu7FollowerConfig(**values)


class FakeBus:
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.messages = []
        self.sent = []
        self.enabled = False
        self.position_rad = math.radians(30)
        self.mode = 2
        self.timeout_ticks = 10_000
        self.shutdown_called = False

    def send(self, message):
        self.sent.append(message)
        data = bytes(message.data)
        if message.arbitration_id == 0x7FF:
            register = data[3]
            value = {8: 3, 7: 0x13, 10: self.mode, 9: self.timeout_ticks}.get(register)
            payload = struct.pack("<f", self.position_rad) if register == 0x50 else struct.pack("<I", value)
            self.messages.append(
                SimpleNamespace(arbitration_id=0x13, data=bytes((3, 0, 0x33, register)) + payload)
            )
        elif data == bytes.fromhex("ff ff ff ff ff ff ff fc"):
            self.enabled = True
            self._status()
        elif data == bytes.fromhex("ff ff ff ff ff ff ff fd"):
            self.enabled = False
        else:
            self._status()

    def _status(self):
        state = 1 if self.enabled else 0
        self.messages.append(SimpleNamespace(arbitration_id=0x13, data=bytes((3 | state << 4,)) + b"\0" * 7))

    def recv(self, timeout):
        return self.messages.pop(0) if self.messages else None

    def shutdown(self):
        self.shutdown_called = True


@pytest.fixture
def fake_can(monkeypatch):
    bus = FakeBus()
    module = SimpleNamespace(
        interface=SimpleNamespace(Bus=lambda **kwargs: bus),
        Message=lambda **kwargs: SimpleNamespace(**kwargs),
    )
    monkeypatch.setattr(
        "lerobot.robots.jaka_zu7_follower.damiao_gripper.importlib.import_module",
        lambda name: module if name == "can" else None,
    )
    return bus


def test_gripper_is_disabled_without_explicit_configuration():
    robot = JAKAZu7Follower(JAKAZu7FollowerConfig(ip_address="10.5.5.100"))
    assert "gripper.pos" not in robot.action_features


def test_gripper_feature_is_available_only_when_configured():
    robot = JAKAZu7Follower(gripper_config())
    assert "gripper.pos" in robot.action_features
    assert "gripper.pos" in robot.observation_features


@pytest.mark.parametrize(
    "changes",
    [
        {"gripper_can_id": None},
        {"gripper_can_id": 0},
        {"gripper_can_interface": "unknown"},
        {"gripper_open_deg": 90.0},
        {"gripper_max_velocity_deg_s": 0.0},
        {"gripper_leader_sign": 0},
        {"gripper_leader_sign": 2},
    ],
)
def test_incomplete_or_invalid_gripper_configuration_is_rejected(changes):
    with pytest.raises(ValueError):
        gripper_config(**changes)


def test_mode_mismatch_never_enables_motor(fake_can):
    fake_can.mode = 1
    gripper = DamiaoGripper(gripper_config())
    with pytest.raises(RuntimeError, match="position-velocity mode"):
        gripper.connect()
    assert not fake_can.enabled
    assert fake_can.shutdown_called
    assert all(message.arbitration_id == 0x7FF for message in fake_can.sent)


@pytest.mark.parametrize(
    "changes",
    [
        {"gripper_control_mode": 4},
        {"gripper_control_mode": 4, "gripper_current_limit_pct": 0},
        {"gripper_control_mode": 4, "gripper_current_limit_pct": 100.01},
        {"gripper_control_mode": 4, "gripper_current_limit_pct": float("nan")},
        {"gripper_control_mode": 2, "gripper_current_limit_pct": 20},
        {"gripper_control_mode": 5},
    ],
)
def test_current_cap_configuration_requires_mode_four_and_valid_percentage(changes):
    with pytest.raises(ValueError):
        gripper_config(**changes)


def test_mode_four_mismatch_never_enables_motor(fake_can):
    gripper = DamiaoGripper(gripper_config(gripper_control_mode=4, gripper_current_limit_pct=15))
    with pytest.raises(RuntimeError, match="force-position mode"):
        gripper.connect()
    assert not fake_can.enabled
    assert all(message.arbitration_id == 0x7FF for message in fake_can.sent)


def test_mode_four_caps_current_on_startup_and_motion(fake_can):
    fake_can.mode = 4
    gripper = DamiaoGripper(
        gripper_config(
            gripper_control_mode=4,
            gripper_current_limit_pct=17.37,
            gripper_max_velocity_deg_s=60,
        )
    )
    gripper.connect()
    gripper.send(40)
    gripper.send(50)
    gripper.move_to_deg(60)
    commands = [message for message in fake_can.sent if message.arbitration_id == 0x303]
    assert len(commands) == 6  # Hold before/after enable, enable frame, and three positions.
    position_commands = [message for message in commands if message.data[:7] != b"\xff" * 7]
    assert len(position_commands) == 5
    for message in position_commands:
        position, speed, current = struct.unpack("<fHH", message.data)
        assert 0 <= math.degrees(position) <= 90
        assert speed == int(math.radians(30) * 100)
        assert current == 1737
    assert math.degrees(struct.unpack("<fHH", position_commands[-1].data)[0]) == pytest.approx(60)
    gripper.disconnect()
    assert fake_can.sent[-1].arbitration_id == 0x303
    assert fake_can.sent[-1].data == bytes.fromhex("ff ff ff ff ff ff ff fd")


def test_absolute_move_rejects_out_of_range_target(fake_can):
    gripper = DamiaoGripper(gripper_config())
    gripper.connect()
    sent = len(fake_can.sent)
    with pytest.raises(RuntimeError, match="outside configured jaw limits"):
        gripper.move_to_deg(91)
    assert len(fake_can.sent) == sent


def test_missing_communication_timeout_never_enables_motor(fake_can):
    fake_can.timeout_ticks = 0
    gripper = DamiaoGripper(gripper_config())
    with pytest.raises(RuntimeError, match="communication timeout"):
        gripper.connect()
    assert not fake_can.enabled
    assert fake_can.shutdown_called


def test_connect_holds_current_angle_and_first_leader_action(fake_can):
    gripper = DamiaoGripper(gripper_config())
    gripper.connect()
    assert fake_can.enabled
    first_position_frame = next(message for message in fake_can.sent if message.arbitration_id == 0x103)
    assert struct.unpack("<f", first_position_frame.data[:4])[0] == pytest.approx(math.radians(30))
    assert gripper.send(40) == pytest.approx(100 / 3)
    assert gripper.send(50) == pytest.approx(100 * 39 / 90)
    assert gripper.read() == pytest.approx(100 / 3, abs=0.001)
    gripper.disconnect()
    assert not fake_can.enabled
    assert fake_can.shutdown_called


def test_decreasing_leader_reading_closes_gripper_from_logged_start(fake_can):
    fake_can.position_rad = math.radians(0.014)
    gripper = DamiaoGripper(gripper_config(gripper_closed_deg=57.2958, gripper_leader_sign=-1))
    gripper.connect()
    gripper.send(93.58)
    initial_target_rad, _ = struct.unpack("<ff", fake_can.sent[-1].data)
    assert math.degrees(initial_target_rad) == pytest.approx(0.014)
    sent = gripper.send(13.64)
    target_rad, speed_rad_s = struct.unpack("<ff", fake_can.sent[-1].data)
    assert math.degrees(target_rad) == pytest.approx(45.8163, abs=0.001)
    assert sent == pytest.approx(79.9644, abs=0.001)
    assert math.degrees(speed_rad_s) == pytest.approx(90)
    gripper.send(100)
    assert struct.unpack("<ff", fake_can.sent[-1].data)[0] == 0


def test_reverse_leader_mapping_clamps_at_closed_limit(fake_can):
    fake_can.position_rad = math.radians(0.014)
    gripper = DamiaoGripper(gripper_config(gripper_closed_deg=57.2958, gripper_leader_sign=-1))
    gripper.connect()
    gripper.send(100)
    assert gripper.send(0) == pytest.approx(100)
    assert math.degrees(struct.unpack("<ff", fake_can.sent[-1].data)[0]) == pytest.approx(57.2958)


def test_out_of_range_start_position_never_enables_motor(fake_can):
    fake_can.position_rad = math.radians(100)
    gripper = DamiaoGripper(gripper_config())
    with pytest.raises(RuntimeError, match="outside configured jaw limits"):
        gripper.connect()
    assert not fake_can.enabled


class FakeUSB2CANSerial:
    def __init__(self):
        self.writes = []
        self.incoming = bytearray()
        self.enabled = False
        self.closed = False

    def write(self, frame):
        frame = bytes(frame)
        self.writes.append(frame)
        can_id = int.from_bytes(frame[13:15], "little")
        data = frame[21:29]
        if can_id == 0x7FF:
            register = data[3]
            values = {7: 0x14, 8: 0x7, 9: 10_000, 10: 2}
            payload = (
                struct.pack("<f", math.radians(30))
                if register == 0x50
                else struct.pack("<I", values[register])
            )
            self._reply(bytes((7, 0, 0x33, register)) + payload)
        else:
            if data == bytes.fromhex("ff ff ff ff ff ff ff fc"):
                self.enabled = True
            elif data == bytes.fromhex("ff ff ff ff ff ff ff fd"):
                self.enabled = False
            self._reply(bytes((7 | int(self.enabled) << 4,)) + b"\0" * 7)
        return len(frame)

    def _reply(self, data):
        self.incoming.extend(b"\xaa\x11\0" + (0x14).to_bytes(4, "little") + data + b"\x55")

    def read(self, size):
        chunk = bytes(self.incoming[:size])
        del self.incoming[:size]
        return chunk

    @property
    def in_waiting(self):
        return len(self.incoming)

    def close(self):
        self.closed = True


def test_legacy_usb2can_frame_matches_official_protocol(monkeypatch):
    fake = FakeUSB2CANSerial()
    monkeypatch.setattr(serial, "Serial", lambda **kwargs: fake)
    bus = DamiaoUSB2CAN("/dev/ttyACM1")
    bus.send(0x7FF, bytes((7, 0, 0x33, 8, 0, 0, 0, 0)))
    assert len(fake.writes[0]) == 30
    assert fake.writes[0][:13] == bytes.fromhex("55 aa 1e 03 01 00 00 00 0a 00 00 00 00")
    assert fake.writes[0][13:15] == b"\xff\x07"
    assert fake.writes[0][21:29] == bytes((7, 0, 0x33, 8, 0, 0, 0, 0))
    reply = bus.recv(0.05)
    assert reply.arbitration_id == 0x14
    assert reply.data[:4] == bytes((7, 0, 0x33, 8))
    bus.shutdown()
    assert fake.closed


def test_legacy_usb2can_gripper_uses_serial_transport(monkeypatch):
    fake = FakeUSB2CANSerial()
    monkeypatch.setattr(serial, "Serial", lambda **kwargs: fake)
    gripper = DamiaoGripper(
        gripper_config(
            gripper_can_channel="/dev/ttyACM1",
            gripper_can_interface="dm_usb2can",
            gripper_can_id=7,
            gripper_feedback_id=0x14,
        )
    )
    gripper.connect()
    assert fake.enabled
    assert gripper.send(40) == pytest.approx(100 / 3)
    assert gripper.read() == pytest.approx(100 / 3, abs=0.001)
    gripper.disconnect()
    assert not fake.enabled
    assert fake.closed


def test_usb2can_ignores_adapter_packets(monkeypatch):
    fake = FakeUSB2CANSerial()
    monkeypatch.setattr(serial, "Serial", lambda **kwargs: fake)
    bus = DamiaoUSB2CAN("unused")
    # A non-CAN packet with plausible motor status must never count as feedback.
    fake.incoming.extend(b"\xaa\x12\0" + (0x14).to_bytes(4, "little") + b"\x17" + b"\0" * 7 + b"\x55")
    fake._reply(b"\x07" + b"\0" * 7)
    assert bus.recv(0.05).data[0] == 0x07


def test_usb2can_does_not_wait_for_64_bytes(monkeypatch):
    fake = FakeUSB2CANSerial()
    monkeypatch.setattr(serial, "Serial", lambda **kwargs: fake)
    bus = DamiaoUSB2CAN("unused")
    fake._reply(b"\x17" + b"\0" * 7)
    original_read = fake.read

    def read(size):
        assert size <= len(fake.incoming), "read waits for missing bytes despite a complete frame"
        return original_read(size)

    fake.read = read
    assert bus.recv(0.05).data[0] == 0x17


def test_connect_consumes_all_startup_replies(fake_can):
    gripper = DamiaoGripper(gripper_config())
    gripper.connect()
    assert not fake_can.messages
    fake_can.enabled = False
    with pytest.raises(RuntimeError, match="[Dd]isabled|not enabled"):
        gripper.send(40)


def test_read_only_probe_queries_status_without_enabling():
    class Bus:
        def __init__(self):
            self.sent = []

        def send(self, can_id, data):
            self.sent.append((can_id, data))

        def recv(self, timeout):
            return SimpleNamespace(arbitration_id=0x14, data=b"\x07" + b"\0" * 7)

    bus = Bus()
    assert read_status(bus, 7, 0x14) == 0
    assert bus.sent == [(0x7FF, bytes((7, 0, 0xCC, 0, 0, 0, 0, 0)))]


def test_usb2can_parses_buffered_frame_before_reading_again(monkeypatch):
    fake = FakeUSB2CANSerial()
    monkeypatch.setattr(serial, "Serial", lambda **kwargs: fake)
    bus = DamiaoUSB2CAN("unused")
    fake._reply(b"\x17" + b"\0" * 7)
    fake._reply(b"\x07" + b"\0" * 7)
    assert bus.recv(0.05).data[0] == 0x17

    def unexpected_read(size):
        raise AssertionError("A buffered frame must not wait for another serial read")

    fake.read = unexpected_read
    assert bus.recv(0.05).data[0] == 0x07
