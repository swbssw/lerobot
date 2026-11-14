import json
import math
import os
import selectors
import shutil
import signal
import subprocess
import sys
import textwrap
import time
from pathlib import Path

import pytest

from examples.jaka import diagnose_step
from examples.jaka.reset_to_reference import REFERENCE_DEG, reset_to_reference
from examples.jaka.teleoperate import run_loop
from lerobot.robots.jaka_zu7_follower import JAKAZu7Follower, JAKAZu7FollowerConfig
from lerobot.robots.jaka_zu7_follower.sdk_bridge import JakaSDKBridge


@pytest.fixture
def sdk_dir(tmp_path):
    (tmp_path / "jkrc.py").write_text(
        textwrap.dedent("""\
            import json
            import os
            import time
            from pathlib import Path

            root = Path(__file__).parent

            class RC:
                def __init__(self, ip):
                    if ip != "10.5.5.100":
                        raise ValueError("wrong controller IP")
                    self.joints = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]

                def record(self, method, *args):
                    with (root / "events.jsonl").open("a") as output:
                        output.write(json.dumps([method, *args]) + "\\n")
                    return (0,)

                def login(self):
                    print("Python SDK log")
                    os.write(1, b"native SDK log\\n")
                    (root / "sdk_connected").touch()
                    (root / "control_connection_lost").unlink(missing_ok=True)
                    return self.record("login")

                def logout(self):
                    (root / "sdk_connected").unlink(missing_ok=True)
                    return self.record("logout")

                def power_on(self):
                    return self.record("power_on")

                def enable_robot(self):
                    return self.record("enable_robot")

                def get_joint_position(self):
                    self.record("get_joint_position")
                    mode_path = root / "mode"
                    mode = mode_path.read_text() if mode_path.exists() else "normal"
                    if mode == "timeout":
                        time.sleep(30)
                    if mode == "slow":
                        time.sleep(0.5)
                    if mode == "crash":
                        os._exit(2)
                    if mode == "exception":
                        raise RuntimeError("read failed")
                    if mode == "error_code":
                        return (-3,)
                    return (0, self.joints)

                def get_robot_state(self):
                    self.record("get_robot_state")
                    state_file = root / "state.json"
                    state = json.loads(state_file.read_text()) if state_file.exists() else [0, 1, 1]
                    return (0, state)

                def is_in_pos(self):
                    self.record("is_in_pos")
                    return (0, True)

                def get_tool_id(self):
                    self.record("get_tool_id")
                    if (root / "control_connection_lost").exists():
                        time.sleep(30)
                    return (0, 0)

                def joint_move_extend(self, joints, mode, block, speed, acc, tol):
                    self.record("joint_move_extend", joints, mode, block, speed, acc, tol)
                    if (root / "control_connection_lost").exists():
                        time.sleep(30)
                    mode_file = root / "mode"
                    behavior = mode_file.read_text() if mode_file.exists() else "normal"
                    if behavior == "move_error":
                        return (-3,)
                    if behavior != "never_reach":
                        self.joints = joints
                    return (0,)

                def motion_abort(self):
                    return self.record("motion_abort")

                def servo_move_enable(self, enable):
                    result = self.record("servo_move_enable", enable)
                    if not enable and (root / "servo_exit_timeout").exists():
                        return (-61,)
                    return result

                def servo_j_extend(self, joints, move_mode, step_num):
                    return self.record("servo_j_extend", joints, move_mode, step_num)

                def servo_j(self, joints, move_mode):
                    return self.record("servo_j", joints, move_mode)

                def servo_move_use_joint_NLF(self, velocity, acceleration, jerk):
                    return self.record("servo_move_use_joint_NLF", velocity, acceleration, jerk)

                def servo_move_use_none_filter(self):
                    return self.record("servo_move_use_none_filter")
            """),
        encoding="utf-8",
    )
    return tmp_path


def read_events(sdk_dir: Path) -> list:
    return [json.loads(line) for line in (sdk_dir / "events.jsonl").read_text().splitlines()]


@pytest.fixture
def servo_mode_query(monkeypatch, sdk_dir):
    responses = [
        {"cmdName": "is_in_servomove", "errorCode": "0", "in_servomove": True},
        {"cmdName": "is_in_servomove", "errorCode": "0", "in_servomove": False},
    ]
    requests = []

    class Connection:
        def __init__(self):
            response = responses.pop(0)
            payload = response if isinstance(response, bytes) else json.dumps(response).encode()
            self.chunks = [payload[:10], payload[10:], b""]

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            pass

        def settimeout(self, _timeout):
            pass

        def sendall(self, request):
            requests.append(json.loads(request))

        def recv(self, _size):
            return self.chunks.pop(0)

    def connect(address, *, timeout):
        assert address == ("10.5.5.100", 10001)
        assert timeout > 0
        if (sdk_dir / "sdk_connected").exists():
            (sdk_dir / "control_connection_lost").touch()
        return Connection()

    monkeypatch.setattr("lerobot.robots.jaka_zu7_follower.sdk_bridge.socket.create_connection", connect)
    return responses, requests


@pytest.mark.parametrize("python", [sys.executable, shutil.which("python3.10")])
def test_bridge_runs_without_lerobot_in_worker_and_ignores_sdk_stdout(sdk_dir, python):
    if python is None:
        pytest.skip("Python 3.10 is not installed")
    with JakaSDKBridge("10.5.5.100", python, str(sdk_dir)) as sdk:
        assert sdk.login() == [0]
        assert sdk.get_joint_position() == [0, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]]
        assert sdk.logout() == [0]
    assert read_events(sdk_dir) == [["login"], ["get_joint_position"], ["logout"]]


def test_worker_cleans_up_servo_and_login_on_eof(sdk_dir):
    with JakaSDKBridge("10.5.5.100", sys.executable, str(sdk_dir)) as sdk:
        sdk.login()
        sdk.servo_move_enable(True)
    assert read_events(sdk_dir) == [
        ["login"],
        ["servo_move_enable", True],
        ["servo_move_enable", False],
        ["logout"],
    ]
    sdk.close()  # Cleanup is idempotent.


def test_reference_teleop_path_configures_nlf_and_sends_servo_j_without_feedback(sdk_dir, tmp_path):
    class Leader:
        def get_action(self):
            return dict.fromkeys(instance.action_features, 30.0)

    instance = JAKAZu7Follower(
        JAKAZu7FollowerConfig(
            ip_address="10.5.5.100",
            sdk_python=sys.executable,
            sdk_dir=str(sdk_dir),
            calibration_dir=tmp_path,
            servo_api="servo_j",
            servo_step_num=1,
            relative_start=True,
            configure_servo_nlf=True,
            max_joint_velocity_deg_s=10,
            max_joint_acceleration_deg_s2=60,
            max_joint_jerk_deg_s3=240,
        )
    )
    instance.connect()
    try:
        # A feedback read in the hot loop would now fail, rather than hiding behind a fast mock.
        (sdk_dir / "mode").write_text("exception")
        run_loop(Leader(), instance, fps=60, duration_s=0.04)
    finally:
        instance.disconnect()
    events = read_events(sdk_dir)
    assert events.count(["get_joint_position"]) == 1  # Only at connection, never per servo tick.
    assert events.index(["servo_move_use_joint_NLF", 10, 60, 240]) < events.index(["servo_move_enable", True])
    commands = [event for event in events if event[0] == "servo_j"]
    assert commands
    for command in commands:
        assert command[1] == pytest.approx([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        assert command[2:] == [0]
    assert events[-2:] == [["servo_move_enable", False], ["logout"]]


def test_jaka_hold_diagnostic_streams_current_pose_without_leader_or_feedback(sdk_dir, monkeypatch):
    monkeypatch.setattr(
        diagnose_step._system,
        "choose",
        lambda: {"sdk_python": sys.executable, "sdk_dir": str(sdk_dir)},
    )
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "diagnose_step.py",
            "--ip=10.5.5.100",
            "--step-deg=0",
            "--hold-s=0.04",
            "--readback-interval-s=0",
            "--nlf-vel=100",
            "--nlf-acc=600",
            "--nlf-jerk=2000",
        ],
    )

    diagnose_step.main()

    events = read_events(sdk_dir)
    assert events.count(["get_joint_position"]) == 1
    assert ["servo_move_use_joint_NLF", 100.0, 600.0, 2000.0] in events
    commands = [event for event in events if event[0] == "servo_j"]
    assert len(commands) >= 2
    assert all(event[1:] == [[0.1, 0.2, 0.3, 0.4, 0.5, 0.6], 0] for event in commands)
    assert events[-2:] == [["servo_move_enable", False], ["logout"]]


def test_jaka_step_diagnostic_rejects_direct_motion_without_explicit_acknowledgement(sdk_dir, monkeypatch):
    monkeypatch.setattr(
        diagnose_step._system,
        "choose",
        lambda: {"sdk_python": sys.executable, "sdk_dir": str(sdk_dir)},
    )
    monkeypatch.setattr(sys, "argv", ["diagnose_step.py", "--ip=10.5.5.100", "--step-deg=0.5"])

    with pytest.raises(SystemExit, match="2"):
        diagnose_step.main()

    assert not (sdk_dir / "events.jsonl").exists()


@pytest.mark.parametrize("mode", ["timeout", "crash"])
def test_bridge_failure_closes_worker_and_never_retries_commands(sdk_dir, mode):
    with JakaSDKBridge("10.5.5.100", sys.executable, str(sdk_dir), timeout_s=0.5) as sdk:
        sdk.login()
        (sdk_dir / "mode").write_text(mode)
        error = TimeoutError if mode == "timeout" else RuntimeError
        with pytest.raises(error, match="did not respond|closed its response pipe"):
            sdk.get_joint_position()
        with pytest.raises(RuntimeError, match="bridge is closed"):
            sdk.get_joint_position()
    assert read_events(sdk_dir).count(["get_joint_position"]) == 1
    assert sdk._process.poll() is not None


def test_worker_reports_sdk_exceptions_and_can_still_logout(sdk_dir):
    with JakaSDKBridge("10.5.5.100", sys.executable, str(sdk_dir)) as sdk:
        sdk.login()
        (sdk_dir / "mode").write_text("exception")
        with pytest.raises(RuntimeError, match="get_joint_position.*read failed"):
            sdk.get_joint_position()
        assert sdk.logout() == [0]


def test_worker_reports_sdk_import_failure(tmp_path):
    (tmp_path / "jkrc.py").write_text('raise ImportError("unsupported SDK binary")')
    with pytest.raises(RuntimeError, match="failed to start.*unsupported SDK binary"):
        JakaSDKBridge("10.5.5.100", sys.executable, str(tmp_path))


def test_driver_uses_bridge_for_feedback_and_motion_and_closes_it(sdk_dir, tmp_path, monkeypatch):
    def reject_parent_import(_name):
        raise AssertionError("The parent must not import jkrc")

    monkeypatch.setattr(
        "lerobot.robots.jaka_zu7_follower.jaka_zu7_follower.importlib.import_module",
        reject_parent_import,
    )
    config = JAKAZu7FollowerConfig(
        ip_address="10.5.5.100",
        sdk_python=sys.executable,
        sdk_dir=str(sdk_dir),
        calibration_dir=tmp_path,
    )
    robot = JAKAZu7Follower(config)
    robot.connect()
    bridge = robot._sdk
    try:
        assert robot.get_observation()["shoulder_pan.pos"] == pytest.approx(5.72957795)
        action = dict.fromkeys(robot.action_features, 0.0)
        action["gripper.pos"] = 50.0
        robot.send_action(action)
    finally:
        robot.disconnect()
    events = read_events(sdk_dir)
    commands = [event for event in events if event[0] == "servo_j_extend"]
    assert len(commands) == 1
    assert len(commands[0][1]) == 6
    assert commands[0][1][0] == pytest.approx(0.1, abs=0.001)
    assert commands[0][2:] == [0, 2]
    assert events[-2:] == [["servo_move_enable", False], ["logout"]]
    assert ["power_on"] not in events
    assert ["enable_robot"] not in events
    with pytest.raises(RuntimeError, match="bridge is closed"):
        bridge.login()
    assert not robot.is_connected


def test_driver_cleans_up_bridge_when_connection_fails(sdk_dir, tmp_path):
    (sdk_dir / "mode").write_text("error_code")
    robot = JAKAZu7Follower(
        JAKAZu7FollowerConfig(
            ip_address="10.5.5.100",
            sdk_python=sys.executable,
            sdk_dir=str(sdk_dir),
            calibration_dir=tmp_path,
        )
    )
    with pytest.raises(RuntimeError, match="get_joint_position.*-3"):
        robot.connect()
    assert read_events(sdk_dir)[-1] == ["logout"]
    assert robot._sdk is None
    assert not robot.is_connected


@pytest.mark.parametrize("phase", ["idle", "pending_call"])
def test_terminal_ctrl_c_allows_driver_to_disconnect_cleanly(sdk_dir, phase):
    script = textwrap.dedent("""\
        import signal
        import sys
        from pathlib import Path
        from lerobot.robots.jaka_zu7_follower import JAKAZu7Follower, JAKAZu7FollowerConfig

        sdk_dir, phase = sys.argv[1:]
        robot = JAKAZu7Follower(JAKAZu7FollowerConfig(
            ip_address="10.5.5.100", sdk_python=sys.executable,
            sdk_dir=sdk_dir, calibration_dir=Path(sdk_dir),
        ))
        robot.connect()
        try:
            if phase == "pending_call":
                (Path(sdk_dir) / "mode").write_text("slow")
            print("ready", flush=True)
            if phase == "pending_call":
                robot.get_observation()
            while True:
                signal.pause()
        except KeyboardInterrupt:
            pass
        finally:
            robot.disconnect()
        """)
    process = subprocess.Popen(
        [sys.executable, "-c", script, str(sdk_dir), phase],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        start_new_session=True,
    )
    try:
        with selectors.DefaultSelector() as selector:
            selector.register(process.stdout, selectors.EVENT_READ)
            assert selector.select(timeout=10), "driver did not become ready"
            assert process.stdout.readline().strip() == "ready"
        if phase == "pending_call":
            deadline = time.monotonic() + 5
            while read_events(sdk_dir).count(["get_joint_position"]) < 2:
                assert time.monotonic() < deadline, "SDK read did not begin"
                time.sleep(0.01)
        # Terminal Ctrl+C targets the foreground process group, not just the parent.
        os.killpg(process.pid, signal.SIGINT)
        _, stderr = process.communicate(timeout=10)
        assert process.returncode == 0, stderr
        assert "Traceback" not in stderr
        assert read_events(sdk_dir)[-2:] == [["servo_move_enable", False], ["logout"]]
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.communicate(timeout=5)


def test_reset_preview_never_changes_robot_state_or_sends_motion(sdk_dir):
    with JakaSDKBridge("10.5.5.100", sys.executable, str(sdk_dir)) as sdk:
        sdk.login()
        reset_to_reference(sdk, REFERENCE_DEG)
    assert read_events(sdk_dir) == [["login"], ["get_robot_state"], ["get_joint_position"], ["logout"]]


def test_reset_skips_servo_exit_when_already_outside_servo_mode(sdk_dir, servo_mode_query):
    (sdk_dir / "servo_exit_timeout").touch()
    responses, requests = servo_mode_query
    responses.pop(0)
    with JakaSDKBridge("10.5.5.100", sys.executable, str(sdk_dir), timeout_s=0.5) as sdk:
        sdk.login()
        reset_to_reference(sdk, REFERENCE_DEG, execute=True)
    events = read_events(sdk_dir)
    assert ["servo_move_enable", False] not in events
    assert sum(event[0] == "joint_move_extend" for event in events) == 1
    assert not (sdk_dir / "control_connection_lost").exists()
    assert events.count(["login"]) == 2
    assert events.count(["logout"]) == 2
    assert requests == [{"cmdName": "is_in_servomove"}]


def test_reset_sends_absolute_radians_with_speed_and_acceleration_limits(sdk_dir, servo_mode_query):
    with JakaSDKBridge("10.5.5.100", sys.executable, str(sdk_dir)) as sdk:
        sdk.login()
        reset_to_reference(sdk, REFERENCE_DEG, execute=True)
    events = read_events(sdk_dir)
    command = next(event for event in events if event[0] == "joint_move_extend")
    assert command[1] == pytest.approx([math.radians(angle) for angle in REFERENCE_DEG])
    assert command[2:4] == [0, False]
    assert command[4:] == pytest.approx([math.radians(20), math.radians(40), 0.0])
    assert events.index(["servo_move_enable", False]) < events.index(command)
    assert events[-2:] == [["motion_abort"], ["logout"]]


@pytest.mark.parametrize("control", ["servo", "joint"])
def test_mode_query_refuses_to_disconnect_during_motion(sdk_dir, servo_mode_query, control):
    _responses, requests = servo_mode_query
    with JakaSDKBridge("10.5.5.100", sys.executable, str(sdk_dir)) as sdk:
        sdk.login()
        if control == "servo":
            sdk.servo_move_enable(True)
        else:
            sdk.joint_move_extend([0.0] * 6, 0, False, 0.1, 0.2, 0.0)
        with pytest.raises(RuntimeError, match="may be controlling motion"):
            sdk.is_in_servomove()
        assert requests == []
        assert ["logout"] not in read_events(sdk_dir)


def test_hung_move_names_failed_operation_and_never_retries(sdk_dir):
    with JakaSDKBridge("10.5.5.100", sys.executable, str(sdk_dir), timeout_s=0.2) as sdk:
        sdk.login()
        (sdk_dir / "control_connection_lost").touch()
        with pytest.raises(TimeoutError, match="joint_move_extend.*did not respond"):
            sdk.joint_move_extend([0.0] * 6, 0, False, 0.1, 0.2, 0.0)
        assert sdk.closed
    assert sum(event[0] == "joint_move_extend" for event in read_events(sdk_dir)) == 1


@pytest.mark.parametrize("failure", ["exit_timeout", "still_servo", "query_error", "invalid_mode", "eof"])
def test_reset_never_moves_when_servo_exit_cannot_be_confirmed(sdk_dir, servo_mode_query, failure):
    responses, _requests = servo_mode_query
    if failure == "exit_timeout":
        (sdk_dir / "servo_exit_timeout").touch()
    elif failure == "still_servo":
        responses[1]["in_servomove"] = True
    elif failure == "query_error":
        responses[0]["errorCode"] = "-61"
    elif failure == "invalid_mode":
        responses[0]["in_servomove"] = "false"
    else:
        responses[0] = b'{"cmdName":'
    with JakaSDKBridge("10.5.5.100", sys.executable, str(sdk_dir)) as sdk:
        sdk.login()
        with pytest.raises(RuntimeError, match="-61|still in streaming|Invalid JAKA|before a complete"):
            reset_to_reference(sdk, REFERENCE_DEG, execute=True)
    assert not any(event[0] == "joint_move_extend" for event in read_events(sdk_dir))


def test_reset_never_moves_if_servo_mode_query_times_out(sdk_dir, monkeypatch):
    def timeout(*_args, **_kwargs):
        raise TimeoutError("servo-mode query timed out")

    monkeypatch.setattr("lerobot.robots.jaka_zu7_follower.sdk_bridge.socket.create_connection", timeout)
    with JakaSDKBridge("10.5.5.100", sys.executable, str(sdk_dir)) as sdk:
        sdk.login()
        with pytest.raises(TimeoutError, match="servo-mode query timed out"):
            reset_to_reference(sdk, REFERENCE_DEG, execute=True)
    assert not any(event[0] in {"joint_move_extend", "servo_move_enable"} for event in read_events(sdk_dir))


@pytest.mark.parametrize("state", [[1, 1, 1], [0, 0, 0], [0, 1, 0]])
def test_reset_refuses_motion_when_emergency_stopped_unpowered_or_disabled(sdk_dir, state):
    (sdk_dir / "state.json").write_text(json.dumps(state))
    with JakaSDKBridge("10.5.5.100", sys.executable, str(sdk_dir)) as sdk:
        sdk.login()
        with pytest.raises(RuntimeError, match="emergency stop released"):
            reset_to_reference(sdk, REFERENCE_DEG, execute=True)
    assert not any(event[0] in {"joint_move_extend", "servo_move_enable"} for event in read_events(sdk_dir))


@pytest.mark.parametrize("mode", ["never_reach", "move_error"])
def test_reset_aborts_motion_on_timeout_or_controller_error(sdk_dir, servo_mode_query, mode):
    (sdk_dir / "mode").write_text(mode)
    with JakaSDKBridge("10.5.5.100", sys.executable, str(sdk_dir)) as sdk:
        sdk.login()
        with pytest.raises((TimeoutError, RuntimeError), match="did not reach|joint_move_extend.*-3"):
            reset_to_reference(sdk, REFERENCE_DEG, execute=True, timeout_s=0.05)
    assert read_events(sdk_dir)[-2:] == [["motion_abort"], ["logout"]]


def test_worker_aborts_joint_motion_on_eof(sdk_dir):
    with JakaSDKBridge("10.5.5.100", sys.executable, str(sdk_dir)) as sdk:
        sdk.login()
        sdk.joint_move_extend([0.0] * 6, 0, False, 0.1, 0.2, 0.0)
    assert read_events(sdk_dir)[-2:] == [["motion_abort"], ["logout"]]


@pytest.mark.parametrize("timeout", [0, -1, float("inf"), float("nan")])
def test_config_rejects_invalid_bridge_timeout(timeout):
    with pytest.raises(ValueError, match="sdk_timeout_s"):
        JAKAZu7FollowerConfig(ip_address="10.5.5.100", sdk_timeout_s=timeout)
