"""Offline checks for leader support and JAKA servo tuning."""

from argparse import ArgumentParser, Namespace

import pytest

from examples.jaka.gravity_comp import PITCH_JOINTS, GravityCompensator, GravityModel
from examples.jaka.teleoperate import (
    LeaderTargetFilter,
    _apply_settings,
    _joint_deadbands,
    _load_settings,
    _max_joint_span,
    _setup_gravity_comp,
    run_loop,
)


class StaticBus:
    def __init__(self):
        self.writes = {}

    def sync_read(self, _register, joints, normalize=False):
        assert not normalize
        return dict.fromkeys(joints, 2048)

    def sync_write(self, register, values, normalize=False):
        assert not normalize
        self.writes[register] = values

    def write(self, _register, _joint, _value, normalize=False):
        assert not normalize

    def enable_torque(self, _joints):
        pass


def test_ids_3_4_6_get_more_support_than_id_2():
    model = GravityModel(
        signs=(1, 1, 1, 1),
        coeffs={joint: [0.0] * (2 * (4 - index)) for index, joint in enumerate(PITCH_JOINTS)},
        intercepts=dict.fromkeys(PITCH_JOINTS, -320.0),
    )
    bus = StaticBus()
    comp = GravityCompensator(
        bus,
        model,
        stiffness={"shoulder_lift": 26.0, "heavy_joint": 32.0, "elbow_flex": 30.0, "wrist_flex": 30.0},
    )
    comp.step()
    goals = bus.writes["Goal_Position"]
    assert goals["shoulder_lift"] > goals["heavy_joint"]
    assert goals["shoulder_lift"] > goals["elbow_flex"] == goals["wrist_flex"] > goals["heavy_joint"]


def test_teleop_settings_keep_fast_motion_limits_and_enable_input_filter():
    settings = _load_settings()
    assert float(settings["max-velocity-deg-s"]) == 100
    assert float(settings["max-acceleration-deg-s2"]) == 600
    assert float(settings["max-jerk-deg-s3"]) == 2000
    assert float(settings["leader-cutoff-hz"]) == 12
    assert [float(value) for value in settings["leader-deadband-deg"].split()] == [
        0.2,
        2.0,
        2.0,
        0.5,
        0.1,
        0.1,
    ]
    assert settings["configure-nlf"].lower() == "true"


def test_joint_deadband_settings_and_scalar_cli_override():
    parser = ArgumentParser()
    parser.add_argument("--leader-deadband-deg", type=float, nargs="+", default=0.0)
    _apply_settings(parser)
    expected = {
        "shoulder_pan.pos": 0.2,
        "heavy_joint.pos": 2.0,
        "shoulder_lift.pos": 2.0,
        "elbow_flex.pos": 0.5,
        "wrist_flex.pos": 0.1,
        "wrist_roll.pos": 0.1,
    }
    assert _joint_deadbands(parser.parse_args([]).leader_deadband_deg) == expected
    assert set(
        _joint_deadbands(parser.parse_args(["--leader-deadband-deg", "0.4"]).leader_deadband_deg).values()
    ) == {0.4}
    with pytest.raises(ValueError, match="one or six"):
        _joint_deadbands((0.2, 2.0))


def test_leader_filter_preserves_first_pose_and_reduces_one_frame_jitter():
    smoothing = LeaderTargetFilter(12)
    first = {"shoulder_lift.pos": 30.0}
    assert smoothing.apply(first, 0.0) == first
    spike = smoothing.apply({"shoulder_lift.pos": 31.0}, 0.008)["shoulder_lift.pos"]
    assert 30.0 < spike < 30.5
    recovered = smoothing.apply(first, 0.016)["shoulder_lift.pos"]
    assert 30.0 < recovered < spike


def test_leader_filter_holds_small_stationary_jitter_but_tracks_fast_motion():
    smoothing = LeaderTargetFilter(12, deadband_deg=0.8)
    baseline = {"shoulder_lift.pos": 30.0}
    assert smoothing.apply(baseline, 0.0) == baseline
    for index, angle in enumerate((30.6, 29.4, 30.6, 29.4), start=1):
        assert smoothing.apply({"shoulder_lift.pos": angle}, index * 0.008) == baseline

    moving = smoothing.apply({"shoulder_lift.pos": 31.8}, 0.04)["shoulder_lift.pos"]
    assert 30.4 < moving < 31.8
    next_moving = smoothing.apply({"shoulder_lift.pos": 32.8}, 0.048)["shoulder_lift.pos"]
    assert moving < next_moving < 32.8


def test_leader_filter_applies_deadband_by_joint_without_filtering_gripper():
    smoothing = LeaderTargetFilter(12, deadband_deg=(0.2, 2.0, 2.0, 0.5, 0.1, 0.1))
    first = {
        "shoulder_pan.pos": 30.0,
        "heavy_joint.pos": 30.0,
        "shoulder_lift.pos": 30.0,
        "elbow_flex.pos": 30.0,
        "wrist_flex.pos": 30.0,
        "wrist_roll.pos": 30.0,
        "gripper.pos": 30.0,
    }
    assert smoothing.apply(first, 0.0) == first
    second = smoothing.apply(dict.fromkeys(first, 30.3), 0.008)
    assert second["shoulder_pan.pos"] > 30.0
    assert second["heavy_joint.pos"] == second["shoulder_lift.pos"] == 30.0
    assert second["elbow_flex.pos"] == 30.0
    assert second["wrist_flex.pos"] > 30.0
    assert second["wrist_roll.pos"] > 30.0
    assert second["gripper.pos"] > second["wrist_flex.pos"]


def test_motion_span_distinguishes_leader_jitter_from_stable_commands():
    raw = [{"shoulder_lift.pos": 30.0}, {"shoulder_lift.pos": 31.0}]
    commanded = [{"shoulder_lift.pos": 30.0}, {"shoulder_lift.pos": 30.0}]
    assert _max_joint_span(raw) == ("shoulder_lift", 1.0)
    assert _max_joint_span(commanded) == ("shoulder_lift", 0.0)


def test_teleop_loop_sends_filtered_targets_without_delaying_first_pose(monkeypatch):
    class Leader:
        def __init__(self):
            self.positions = iter((30.0, 30.6, 31.8))

        def get_action(self):
            return {"shoulder_lift.pos": next(self.positions)}

    class Follower:
        def __init__(self):
            self.commands = []

        def send_action(self, action):
            self.commands.append(dict(action))
            return action

    timestamps = iter(
        (
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.008,
            0.008,
            0.008,
            0.008,
            0.008,
            0.016,
            0.016,
            0.016,
            0.016,
            0.016,
            0.03,
        )
    )
    monkeypatch.setattr("examples.jaka.teleoperate.time.monotonic", lambda: next(timestamps))
    monkeypatch.setattr("examples.jaka.teleoperate.time.sleep", lambda _: None)
    follower = Follower()

    run_loop(Leader(), follower, fps=125, duration_s=0.025, leader_cutoff_hz=12, leader_deadband_deg=0.8)

    assert follower.commands[0] == {"shoulder_lift.pos": 30.0}
    assert follower.commands[1] == follower.commands[0]
    assert 30.0 < follower.commands[2]["shoulder_lift.pos"] < 30.5


def test_teleop_wires_id3_support_into_compensator():
    class Leader:
        bus = StaticBus()

    args = Namespace(
        gravity_model="examples/jaka/gravity_model.json",
        gravity_p=60,
        gravity_i=0,
        gravity_margin_heavy_joint=45.0,
        gravity_margin_shoulder_lift=160.0,
        gravity_margin_elbow_flex=55.0,
        gravity_margin_wrist_flex=50.0,
        gravity_stiffness=32.0,
        gravity_stiffness_shoulder_lift=26.0,
        gravity_stiffness_elbow_flex=30.0,
        gravity_stiffness_wrist_flex=30.0,
        gravity_anti_droop=True,
        gravity_anti_droop_gain=1.0,
        gravity_anti_droop_kp=30.0,
        gravity_anti_droop_threshold=5.0,
        gravity_anti_droop_max=400.0,
    )
    comp = _setup_gravity_comp(Leader(), args)
    assert comp.stiffness == {
        "heavy_joint": 32.0,
        "shoulder_lift": 26.0,
        "elbow_flex": 30.0,
        "wrist_flex": 30.0,
    }
    assert comp.margin == {
        "heavy_joint": 45.0,
        "shoulder_lift": 160.0,
        "elbow_flex": 55.0,
        "wrist_flex": 50.0,
    }
