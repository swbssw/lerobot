"""Standalone SDK worker: must remain compatible with Python 3.10 and use only stdlib."""

import importlib
import json
import math
import os
import signal
import socket
import sys
from typing import Any, TextIO

SDK_METHODS = frozenset(
    {
        "login",
        "logout",
        "power_on",
        "enable_robot",
        "get_joint_position",
        "get_robot_state",
        "get_robot_status",
        "get_tool_id",
        "is_in_pos",
        "joint_move_extend",
        "motion_abort",
        "servo_move_enable",
        "servo_j_extend",
        "servo_j",
        "servo_move_use_joint_NLF",
        "servo_move_use_none_filter",
    }
)


def _success(result: Any) -> bool:
    return isinstance(result, (tuple, list)) and bool(result) and result[0] == 0


def _sanitize(value: Any) -> Any:
    """Recursively replace non-finite floats with None so json.dumps(allow_nan=False) works.

    Some SDK results (e.g. get_robot_status monitor data) can contain NaN/inf, which
    json.dumps(allow_nan=False) rejects. Replace them with JSON null (None) instead.
    """
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, (tuple, list)):
        return [_sanitize(item) for item in value]
    return value


def _respond(output: TextIO, payload: dict[str, Any]) -> None:
    output.write(json.dumps(payload, allow_nan=False) + "\n")
    output.flush()


def serve(output: TextIO, ip_address: str) -> None:
    robot: Any = None
    logged_in = False
    servo_enabled = False
    motion_active = False
    try:
        try:
            sdk = importlib.import_module("jkrc")
            robot = sdk.RC(ip_address)
        except Exception as exc:
            _respond(output, {"error": f"{type(exc).__name__}: {exc}"})
            return
        _respond(output, {"ready": True})
        for line in sys.stdin:
            try:
                request = json.loads(line)
                method = request["method"]
                if method not in SDK_METHODS:
                    raise ValueError(f"Unsupported SDK method: {method}")
                args = request["args"]
                if method == "logout" and motion_active:
                    abort_result = robot.motion_abort()
                    if not _success(abort_result):
                        raise RuntimeError(f"Could not stop joint motion before logout: {abort_result}")
                    motion_active = False
                if method == "joint_move_extend":
                    # Even a failed response may follow a command reaching the controller.
                    motion_active = True
                result = getattr(robot, method)(*args)
                if _success(result):
                    if method == "login":
                        logged_in = True
                    elif method == "logout":
                        logged_in = False
                    elif method == "servo_move_enable":
                        servo_enabled = bool(args[0])
                    elif method == "motion_abort":
                        motion_active = False
                response = {"result": _sanitize(result)}
            except Exception as exc:
                response = {"error": f"{type(exc).__name__}: {exc}"}
            _respond(output, response)
    except BrokenPipeError:
        pass
    finally:
        # EOF also covers loss of the parent process. These are best-effort SDK
        # calls, not a substitute for controller-side communication-loss handling.
        if robot is not None:
            if motion_active:
                try:
                    robot.motion_abort()
                except Exception as exc:
                    print(f"JAKA worker motion stop failed: {exc}", file=sys.stderr)
            if servo_enabled:
                try:
                    robot.servo_move_enable(False)
                except Exception as exc:
                    print(f"JAKA worker servo cleanup failed: {exc}", file=sys.stderr)
            if logged_in:
                try:
                    robot.logout()
                except Exception as exc:
                    print(f"JAKA worker logout failed: {exc}", file=sys.stderr)


def _open_output(channel: str) -> Any:
    """Open the response channel: an inherited pipe fd on POSIX, a TCP port on Windows."""
    if os.name == "nt":
        sock = socket.create_connection(("127.0.0.1", int(channel)))
        return sock.makefile("w", buffering=1, encoding="utf-8", newline="\n")
    return os.fdopen(int(channel), "w", buffering=1, encoding="utf-8")


def main() -> None:
    if os.name == "nt":
        # Ctrl+C is still delivered to this subprocess on Windows despite the parent's
        # start_new_session=True (CREATE_NEW_PROCESS_GROUP), which killed the worker
        # mid-request and broke clean shutdown. Ignore Ctrl+C here so it reaches only
        # the parent, which then stops the worker cleanly over stdin/EOF.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
    # Native SDK code writes directly to fd 1. Respond on a separate channel (an
    # inherited pipe fd on POSIX, a socket on Windows) so native prints cannot
    # corrupt the protocol.
    os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
    sdk_dir = sys.argv[3] if len(sys.argv) > 3 else ""
    if os.name == "nt" and sdk_dir and hasattr(os, "add_dll_directory"):
        # Let jkrc.pyd find its sibling jakaAPI.dll on Windows.
        os.add_dll_directory(sdk_dir)
    with _open_output(sys.argv[1]) as output:
        serve(output, sys.argv[2])


if __name__ == "__main__":
    main()
