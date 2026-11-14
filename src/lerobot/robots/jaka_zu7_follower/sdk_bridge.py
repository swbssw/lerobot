"""Run the vendor SDK with a separate Python interpreter over private pipes."""

import json
import math
import os
import selectors
import socket
import subprocess
import threading
import time
from contextlib import suppress
from pathlib import Path
from typing import Any


class JakaSDKBridge:
    """Expose the driver's SDK calls while keeping jkrc out of the LeRobot process."""

    def __init__(
        self, ip_address: str, python: str, sdk_dir: str | None = None, timeout_s: float = 5.0
    ) -> None:
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("SDK bridge timeout must be finite and positive")
        self._timeout_s = timeout_s
        self._ip_address = ip_address
        self._lock = threading.RLock()
        self._logged_in = False
        self._servo_active = False
        self._joint_motion_active = False
        self._buffer = b""
        self._closed = False
        self._process: subprocess.Popen[bytes] | None = None
        self._selector = selectors.DefaultSelector()
        self._sock: socket.socket | None = None
        self._listener: socket.socket | None = None
        self._response_fd: int | None = None
        env = os.environ.copy()
        sdk_dir = sdk_dir or env.get("JAKA_SDK_DIR")
        if sdk_dir:
            sdk_dir = str(Path(sdk_dir).expanduser().resolve())
            # The worker needs only the SDK and the standard library. Do not leak the
            # parent's PYTHONPATH (which may contain Python 3.12 packages) into it.
            env["PYTHONPATH"] = sdk_dir
            env["JAKA_SDK_DIR"] = sdk_dir
            # POSIX only: the SDK shared library sits next to jkrc. On Windows the
            # worker locates jakaAPI.dll via os.add_dll_directory(sdk_dir).
            env["LD_LIBRARY_PATH"] = os.pathsep.join(filter(None, (sdk_dir, env.get("LD_LIBRARY_PATH", ""))))
        env.pop("PYTHONHOME", None)
        worker = Path(__file__).with_name("sdk_worker.py")
        # Windows cannot inherit a pipe fd (pass_fds is POSIX-only) and select() cannot
        # watch a pipe handle. Use a localhost socket the worker connects back to; the
        # port number travels via argv instead of a file descriptor.
        if os.name == "nt":
            self._listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._listener.bind(("127.0.0.1", 0))
            self._listener.listen(1)
            channel = str(self._listener.getsockname()[1])
            popen_kwargs: dict[str, Any] = {
                "stdin": subprocess.PIPE,
                "stdout": subprocess.DEVNULL,
                "env": env,
                "start_new_session": True,
            }
        else:
            self._response_fd, response_writer = os.pipe()
            channel = str(response_writer)
            popen_kwargs = {
                "stdin": subprocess.PIPE,
                "stdout": subprocess.DEVNULL,
                "pass_fds": (response_writer,),
                "env": env,
                "start_new_session": True,
            }
        try:
            # Ctrl+C belongs to the foreground LeRobot process. Let it stop the worker
            # through SDK cleanup/EOF rather than interrupting both processes in the
            # middle of the shutdown exchange.
            self._process = subprocess.Popen(
                [python, "-u", "-S", str(worker), channel, ip_address, sdk_dir or ""],
                **popen_kwargs,
            )
        except BaseException:
            self.close()
            raise
        finally:
            if os.name != "nt":
                os.close(response_writer)
        try:
            if os.name == "nt":
                self._listener.settimeout(self._timeout_s)
                self._sock, _ = self._listener.accept()
                self._sock.settimeout(None)
                self._selector.register(self._sock, selectors.EVENT_READ)
            else:
                self._selector.register(self._response_fd, selectors.EVENT_READ)
            response = self._receive()
            if response.get("ready") is not True:
                raise RuntimeError(f"JAKA SDK bridge failed to start: {response.get('error', response)}")
        except BaseException:
            self.close()
            raise

    def _read_chunk(self) -> bytes:
        if self._sock is not None:
            return self._sock.recv(65536)
        if self._response_fd is None:
            raise RuntimeError("JAKA SDK bridge has no response channel")
        return os.read(self._response_fd, 65536)

    def _receive(self) -> dict[str, Any]:
        deadline = time.monotonic() + self._timeout_s
        while b"\n" not in self._buffer:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not self._selector.select(remaining):
                raise TimeoutError(f"JAKA SDK bridge did not respond within {self._timeout_s}s")
            chunk = self._read_chunk()
            if not chunk:
                raise RuntimeError("JAKA SDK bridge exited or closed its response pipe")
            self._buffer += chunk
        line, self._buffer = self._buffer.split(b"\n", 1)
        response = json.loads(line)
        if not isinstance(response, dict):
            raise RuntimeError("JAKA SDK bridge returned an invalid response")
        return response

    def _call(self, method: str, *args: Any) -> Any:
        with self._lock:
            if self._closed or self._process is None or self._process.stdin is None:
                raise RuntimeError("JAKA SDK bridge is closed")
            request = json.dumps({"method": method, "args": args}, allow_nan=False).encode() + b"\n"
            if method == "joint_move_extend":
                self._joint_motion_active = True
            elif method == "servo_move_enable" and args[0]:
                self._servo_active = True
            try:
                self._process.stdin.write(request)
                self._process.stdin.flush()
                response = self._receive()
            except TimeoutError as exc:
                self.close()
                raise TimeoutError(f"JAKA SDK {method}: {exc}") from exc
            except BaseException:
                # A timed-out command may already have reached the controller. Never
                # retry it or accept another command with an ambiguous response.
                self.close()
                raise
            if "error" in response:
                raise RuntimeError(f"JAKA SDK {method} failed in worker: {response['error']}")
            result = response["result"]
            if isinstance(result, (list, tuple)) and result and result[0] == 0:
                if method == "login":
                    self._logged_in = True
                elif method == "logout":
                    self._logged_in = False
                elif method == "servo_move_enable":
                    self._servo_active = bool(args[0])
                elif method == "motion_abort":
                    self._joint_motion_active = False
            return result

    def login(self) -> Any:
        return self._call("login")

    def logout(self) -> Any:
        return self._call("logout")

    def power_on(self) -> Any:
        return self._call("power_on")

    def enable_robot(self) -> Any:
        return self._call("enable_robot")

    def get_joint_position(self) -> Any:
        return self._call("get_joint_position")

    def get_robot_state(self) -> Any:
        return self._call("get_robot_state")

    def get_robot_status(self) -> Any:
        return self._call("get_robot_status")

    def is_in_pos(self) -> Any:
        return self._call("is_in_pos")

    def get_tool_id(self) -> Any:
        """Read a controller setting to check SDK command-channel responsiveness."""
        return self._call("get_tool_id")

    def is_in_servomove(self) -> tuple[int, bool]:
        """Query streaming mode using the documented read-only TCP command.

        SDK 2.1.14's Python binding lacks this query. This controller cannot
        service the SDK command channel and an extra TCP control connection
        concurrently. Log out before querying, close TCP, then log in again.
        This operation is only allowed before starting motion through this bridge.
        """
        with self._lock:
            if self._closed:
                raise RuntimeError("JAKA SDK bridge is closed")
            if self._servo_active or self._joint_motion_active:
                raise RuntimeError("Cannot query servo mode while this bridge may be controlling motion")
            reconnect = self._logged_in
            if reconnect:
                result = self.logout()
                if not isinstance(result, (list, tuple)) or not result or result[0] != 0:
                    raise RuntimeError(f"Cannot release SDK connection before servo-mode query: {result}")
            mode = self._query_servo_mode()
            if reconnect:
                result = self.login()
                if not isinstance(result, (list, tuple)) or not result or result[0] != 0:
                    raise RuntimeError(f"Cannot restore SDK connection after servo-mode query: {result}")
            return 0, mode

    def _query_servo_mode(self) -> bool:
        """Use a separate TCP connection only while the SDK is logged out."""
        deadline = time.monotonic() + self._timeout_s
        with socket.create_connection((self._ip_address, 10001), timeout=self._timeout_s) as connection:
            connection.settimeout(max(0.001, deadline - time.monotonic()))
            connection.sendall(b'{"cmdName":"is_in_servomove"}\r\n')
            response = bytearray()
            while len(response) < 16384:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("JAKA servo-mode query timed out; no reset motion sent")
                connection.settimeout(remaining)
                chunk = connection.recv(4096)
                if not chunk:
                    raise RuntimeError("JAKA closed the servo-mode query before a complete response")
                response.extend(chunk)
                try:
                    result = json.loads(response)
                except (ValueError, UnicodeDecodeError):
                    continue
                if (
                    not isinstance(result, dict)
                    or result.get("cmdName") != "is_in_servomove"
                    or str(result.get("errorCode")) != "0"
                    or not isinstance(result.get("in_servomove"), bool)
                ):
                    raise RuntimeError(f"Invalid JAKA servo-mode response: {result}")
                return result["in_servomove"]
            raise RuntimeError("JAKA servo-mode response exceeded the size limit")

    def joint_move_extend(
        self, joints_rad: list[float], move_mode: int, is_block: bool, speed: float, acc: float, tol: float
    ) -> Any:
        return self._call("joint_move_extend", joints_rad, move_mode, is_block, speed, acc, tol)

    def motion_abort(self) -> Any:
        return self._call("motion_abort")

    def servo_move_enable(self, enable: bool) -> Any:
        return self._call("servo_move_enable", enable)

    def servo_j_extend(self, joints_rad: list[float], move_mode: int, step_num: int) -> Any:
        return self._call("servo_j_extend", joints_rad, move_mode, step_num)

    def servo_j(self, joints_rad: list[float], move_mode: int) -> Any:
        return self._call("servo_j", joints_rad, move_mode)

    def servo_move_use_joint_NLF(self, max_vr: float, max_ar: float, max_jr: float) -> Any:  # noqa: N802
        return self._call("servo_move_use_joint_NLF", max_vr, max_ar, max_jr)

    def servo_move_use_none_filter(self) -> Any:
        return self._call("servo_move_use_none_filter")

    @property
    def closed(self) -> bool:
        """Whether close has already shut down communication and reaped the worker."""
        return self._closed

    def close(self) -> None:
        """Close stdin so the worker cleans up, then reap it with a bounded wait."""
        if self._closed:
            return
        self._closed = True
        if self._process is not None:
            if self._process.stdin is not None:
                with suppress(OSError):
                    self._process.stdin.close()
            try:
                self._process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self._process.terminate()
                try:
                    self._process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    self._process.kill()
                    self._process.wait()
        self._selector.close()
        if self._sock is not None:
            self._sock.close()
        if self._listener is not None:
            self._listener.close()
        if self._response_fd is not None:
            os.close(self._response_fd)

    def __enter__(self) -> "JakaSDKBridge":
        return self

    def __exit__(self, *_args: Any) -> None:
        self.close()
