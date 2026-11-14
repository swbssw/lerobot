# JAKA Zu 7 Follower

This driver exposes the six JAKA joints as a LeRobot follower. Its joint keys match the six arm joints in the customized SO-ARM101 leader in this repository: `shoulder_pan`, `heavy_joint`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, and `wrist_roll`. The leader's `gripper.pos` action is ignored unless an external Damiao J4340-2EC gripper is explicitly configured.

## Optional J4340-2EC gripper

The seventh actuator is an external CAN motor, not a seventh JAKA axis. The SO-101 leader provides `gripper.pos` as 0-100%. When enabled, the follower adds `gripper.pos` to its action and observation features, so standard LeRobot recording can include the gripper. The motor feedback is reported in the same 0-100% jaw range; it is read from the motor's position register, not inferred from the command.

Use the [Damiao product repository](https://gitee.com/kit-miao/damiao) and the J4340-2EC manual to identify the motor variant, CAN command ID (ESC_ID), feedback ID (MST_ID), bitrate, and stored control mode. The driver supports position-velocity mode (CTRL_MODE=2, default) or force-position hybrid mode (CTRL_MODE=4), and requires classic CAN at 1 Mbps, a command ID of 1-15, an enabled communication-loss timeout of at most 2 seconds (TIMEOUT register 1-40000 in 50 us ticks), and an explicitly confirmed adapter. It checks these motor registers before enabling the gripper and never writes the motor configuration or assigns an ID. Do not infer the command ID from its role as the seventh actuator. The Damiao legacy USB2CAN uses `--gripper-can-interface=dm_usb2can` and a 921600-baud USB serial port; it is not SLCAN. This transport follows the [official u2can Python example](https://gitee.com/kit-miao/motor-sdk/tree/master/Python%E4%BE%8B%E7%A8%8B/u2can). SocketCAN and SLCAN remain available for compatible adapters.

Measure the motor output angle at the two mechanical jaw limits and provide those angles as `--gripper-open-deg` and `--gripper-closed-deg`. The current motor angle must already lie between them. The startup target holds the measured motor angle, and the first leader gripper reading anchors relative motion. Commands are clamped to the configured jaw limits. Confirm jaw direction and clearance before enabling motion. The gripper velocity cap is separate from the JAKA arm velocity cap.

On the legacy Damiao USB2CAN adapter, use the read-only probe first, with JAKA teleoperation and the DM debugging tool stopped:

```bash
python examples/jaka/probe_gripper.py --port=/dev/ttyACM1 --can-id=0x7 --feedback-id=0x14
```

This sends only parameter-read CAN frames and does not enable or move the motor. The USB2CAN adapter must be in CAN forwarding mode. A timeout indicates an interface/mode, power, wiring, CAN bitrate, or ID issue; do not enable motion until resolved. On the observed device, the DM debugging tool showed CAN ID `0x7`, Master ID `0x14`, position-velocity mode `2`, and CAN timeout `0`. Set the motor timeout to a nonzero value (for example 10000 ticks = 0.5 s), save it in the debugging tool, power cycle, and confirm the new value with the probe before starting teleoperation.

For a SocketCAN or SLCAN adapter, install the optional CAN dependency with `uv sync --extra damiao` (or `pip install -e '.[damiao]'` in an existing environment). The legacy Damiao USB2CAN backend needs only pyserial, which is already a LeRobot dependency. Once the adapter, IDs, mode, timeout and jaw angles are confirmed, append these flags to `examples/jaka/teleoperate.py` or supply the matching `--robot.gripper_*` settings to `lerobot-teleoperate`:

```text
--gripper-can-channel=CAN_CHANNEL
--gripper-can-interface=dm_usb2can_OR_socketcan_OR_slcan
--gripper-can-id=COMMAND_ID
--gripper-feedback-id=FEEDBACK_ID
--gripper-open-deg=MEASURED_OPEN_ANGLE
--gripper-closed-deg=MEASURED_CLOSED_ANGLE
--gripper-max-velocity-deg-s=150
```

### Gripper speed and current limit

The dedicated teleoperation script defaults to a gripper speed of 150 degrees/s. When a gripper channel is supplied,
its default open/closed angles are 0/120 degrees; startup still holds the measured position.
Replace an explicit `--gripper-max-velocity-deg-s=10`, `30`, or `90` with
`--gripper-max-velocity-deg-s=150`; explicit arguments override defaults.
For the 0-to-120-degree range, nominal full travel is about 1.33 seconds
before acceleration and load effects. JAKA arm speed settings are independent.

For a per-command current cap, stop teleoperation and disable the motor in the Damiao
tool, select **force-position hybrid / PVT (CTRL_MODE=4)**, then **Write Parameters**.
The manual says this resets the driver and preserves the mode after power cycling.
Close the tool and check `Control mode: 4` with `probe_gripper.py` before use.
Add `--gripper-control-mode=4 --gripper-current-limit-pct=P` to teleoperation,
replacing `P` with a measured/calibrated percentage. Mode 4 requires an explicit cap;
mode 2 rejects this option. A mode mismatch is rejected before enabling the motor.
Every mode-4 position command, including the startup hold, carries the cap.

The [official J4340-2EC manual](https://github.com/dmBots/DM-J4340-2EC/tree/master/manual)
(V1.1, 2026-09-08, pages 12–13 and 16) specifies a position float in radians,
a uint16 speed scaled by 100, and a uint16 current fraction scaled by 10000,
on CAN ID `0x300 + motor ID`. The fraction is relative to **driver maximum phase
current**, not motor rated or peak current. The cap saturates the current request;
it does not freeze the target or measure jaw force. Contact can prevent reaching
the target, and removing the object can allow motion to resume toward that target.
The cap applies in both opening and closing directions and is not an impact-force guarantee.

For the requested two-sided total of 40 N and an approximately 1.6 cm lever arm,
the ideal motor torque is about 0.64 N·m when the two sides together contribute
the specified force. Linkages and friction change this value.
Using the measured motor parameters in this setup (`Kt_out≈3.865 N·m/A` and
`Imax≈10.261 A`) gives a theoretical current cap of about `1.61%` for 40 N.
The manual lists separate 24 V and 48 V electrical specifications and does not
provide a universal percentage for this mechanism. Obtain the actual driver's
maximum phase current and output torque/current relation, then validate with a
force gauge. The code does not claim a calibrated 40 N limit.

### Independent gripper reset

For one-command power/enable and reset, use `examples/jaka/initialize.py`:

```bash
# Read-only preview: no power, enable, clear-fault, or motion commands.
python examples/jaka/initialize.py --ip=10.5.5.100
# Execute: JAKA power/enable, open gripper, reset JAKA, then hold gripper.
python examples/jaka/initialize.py --ip=10.5.5.100 --execute
```

The external J4340 supply must already be on. Stop all other control programs first.
The script checks emergency-stop/power/enable feedback, opens the jaw to 0 degrees
(releasing any held object), resets JAKA to `[0, 90, 0, 90, 180, 0]`, then reconnects
the gripper and sends its measured hold angle at 50 Hz. The jaw is disabled during
the JAKA reset and re-enabled for the final hold. The script stays running to maintain
that hold; with the 10000-tick watchdog it cannot stay enabled after communication ends.
Ctrl+C disables the gripper and frees its port; JAKA remains powered/enabled.
Then start teleoperation, which enables the gripper again. Do not run teleoperation
alongside the hold program on the same adapter. To skip the final hold, add
`--exit-after-reset`. All reset speed/angle/current/port options below are shared.
The SO leader is not powered or repositioned by initialization; open its jaw before teleoperation.

On a new gripper connection only, status `D` triggers one documented `FF FF FF FF FF FF FF FB`
clear-fault command on the configured mode ID. Startup requires disabled feedback after
clearing, re-reads the actual angle, stages a hold target, and verifies enabled feedback.
It never sends the `FE` zero-position command, changes timeout, or clears other faults.
Persistent D, no reply, or another fault aborts startup. D during teleoperation/holding
still stops the session without automatically clearing/re-enabling. This follows the
[official manual](https://github.com/dmBots/DM-J4340-2EC/tree/master/manual), p17;
firmware behavior and physical restart still require hardware verification.

`reset_to_reference.py` now opens the gripper before resetting the six JAKA axes
by default. It uses this setup's stable HDSC serial path, IDs 0x7/0x14, mode 4,
0-to-120-degree limits, 90 degrees/s and the existing uncalibrated 1.61% current cap.
Override these with `--gripper-port`, `--gripper-can-id`, `--gripper-feedback-id`,
`--gripper-control-mode`, `--gripper-open-deg`, `--gripper-closed-deg`,
`--gripper-speed-deg-s`, `--gripper-current-limit-pct`, and `--gripper-timeout-s`.
Without `--execute`, both devices are previewed without enabling or moving them.
Execution checks JAKA readiness, exits streaming mode, opens the gripper and disables
it after measured arrival, then rechecks JAKA readiness before the joint move.
A gripper failure blocks the JAKA move. The gripper still resets when JAKA is
already at the reference pose. Opening releases held objects.
Use `--skip-gripper` explicitly for an arm-only reset (also when no gripper is attached).

For an independent gripper reset, `reset_gripper.py` never connects to JAKA. Stop teleoperation
and other programs using the adapter, and preview the target first:

```bash
python examples/jaka/reset_gripper.py \
  --port=/dev/serial/by-id/usb-HDSC_CDC_Device_00000000050C-if00 \
  --can-id=0x7 --feedback-id=0x14 \
  --open-deg=0 --closed-deg=120 --target-deg=0
```

Preview only reads the mode, position, and status. Add `--execute` to open the
jaw at up to 90 degrees/s (opening releases any held object). With the motor in
mode 4, also supply `--control-mode=4 --current-limit-pct=P` with the same
calibrated cap. Execution verifies the mode, starts by holding the measured
position, streams the target, verifies measured arrival, then disables the motor.
Timeout, feedback errors, and Ctrl+C also attempt disable. Completion does not
leave a powered hold. No physical force or movement has been validated by the
software tests.

All six required gripper settings must be provided together. With none supplied, the existing six-axis teleoperation behaves as before. Stop other processes using the adapter before running the follower.

Use `--gripper-leader-sign=-1` when closing the leader decreases its reported
percentage (on this setup, open is about 94% and closing approaches 0%). The default
`+1` retains the increasing-percentage mapping. This sign changes only the leader
delta; it does not swap the measured motor open/closed limits or reverse the
recorded follower opening range (0% open, 100% closed). For example, from a motor
start of 0.014 degrees and leader start of 93.58%, a leader reading of 13.64% with
sign -1 commands about 45.816 degrees within a 0-to-57.2958-degree jaw range.
Startup still holds the measured motor position. Keep both jaws at corresponding
starting openings. Relative startup and a less-than-100% leader excursion may
not cover the entire mechanical jaw range; do not enlarge physical limits to compensate.

The probe also requests motor status using the read-only `0xCC` refresh frame:
`0` means disabled, `1` means enabled, and `0xD` means communication lost.
Disabled is expected after normal shutdown; the probe itself never enables the motor.
During teleoperation, look for `J4340 enabled: feedback state=1` and the periodic
`Gripper: leader=... command=... clamped=... motor feedback state=1` log.
The command is not measured position. If leader readings change while the command
stays at the open limit with `clamped=True`, check leader opening/closing direction
before changing the mapping. If feedback reports disabled or a fault, teleoperation
raises an error and exits rather than automatically re-enabling.
Legacy USB2CAN reception accepts only `CMD=0x11` CAN packets and consumes buffered
frames without waiting to fill 64 bytes. Startup consumes each command's reply before
sending the next command. Leader gravity compensation is initialized before enabling
the follower to avoid consuming the gripper's communication timeout during setup.

The robot features use degrees to match the SO-ARM101 leader. The `jkrc` Python SDK returns joint positions in radians and accepts radians for `servo_j_extend`; the driver converts feedback and commands at the SDK boundary. This is the Python SDK interface, not JAKA's lower-level TCP JSON protocol, whose servo joint values use degrees. The driver filters each target with configured joint velocity and acceleration limits, then sends an absolute `servo_j_extend` target. The default `servo_step_num=2` spans two 8 ms controller cycles, which is close to the default 60 Hz LeRobot loop.

`joint_signs` can reverse individual axes using `-1`, and `joint_offsets_deg` can align the JAKA zero positions with the SO-ARM101 leader. Both settings contain six values in JAKA joint order. Defaults are all `1` and `0`; measure signs and offsets on the real setup before higher-speed use.

## Install the vendor SDK on Ubuntu

Use the vendor's Linux x86_64 Python SDK from JAKA SDK 2.1.14. Keep `jkrc.so` and `libjakaAPI.so` together in the SDK's `Linux/python3/x86_64-linux-gnu` directory. This repository does not redistribute those vendor binaries.

LeRobot requires Python 3.12 or later, while some vendor `jkrc.so` builds only work with older Python versions. If the SDK imports with Python 3.10 but fails with Python 3.12 (for example, `Py_TPFLAGS_HAVE_GC flag but has no traverse function`), use the subprocess mode below. Keep the LeRobot environment on Python 3.12; the SDK interpreter does not need LeRobot, PyTorch, or additional Python packages.

### SDK subprocess (Python 3.10)

Install the checkout in the active LeRobot Python 3.12 environment:

```bash
pip install -e ".[feetech]"
```

Find a Python executable that can import the vendor SDK, for example `/usr/bin/python3.10`. Set `--robot.sdk_python` to that executable and `--robot.sdk_dir` to the directory containing the two `.so` files. The driver starts and stops the worker automatically; no separate server or network port is needed. The SDK and dynamic library search paths are set for the worker, so you do not need to export them into the LeRobot environment. `JAKA_SDK_DIR` is also accepted when `sdk_dir` is omitted.

From the repository root, test login and joint feedback without powering on, enabling the robot, or entering servo mode (replace the IP and paths for your setup):

```bash
python examples/jaka/read_state.py \
  --ip=10.5.5.100 \
  --sdk-python=/usr/bin/python3.10 \
  --sdk-dir="$HOME/Downloads/jaka_sdk/Linux/python3/x86_64-linux-gnu"
```

Requests use private subprocess pipes. SDK logs go to stderr and cannot corrupt the responses. `sdk_timeout_s` (default 5 seconds) bounds worker startup and each SDK response; it is an error timeout, not the servo cycle. On timeout or worker exit, the bridge closes and does not replay commands. On normal shutdown or stdin EOF, the worker attempts to disable servo mode and log out. A blocked or crashed SDK may prevent that cleanup, so controller-side communication-loss handling and the physical emergency stop remain necessary. The bridge does not provide hard real-time scheduling; verify the loop rate on your setup.

The worker starts in its own session so terminal Ctrl+C interrupts LeRobot, which then shuts down the worker through SDK calls or stdin EOF. If an interrupted SDK call already closed the bridge, disconnect does not send further commands to the closed pipe.

### SDK in the LeRobot process

If your vendor binary is compatible with the LeRobot Python version, omit `sdk_python` and `sdk_dir` to use the original in-process mode:

```bash
export JAKA_SDK_DIR="$HOME/Downloads/jaka_sdk/Linux/python3/x86_64-linux-gnu"
export PYTHONPATH="$JAKA_SDK_DIR${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$JAKA_SDK_DIR${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
python -c 'import jkrc; print("JAKA SDK import OK")'
```

Confirm the JAKA controller firmware is supported by the installed SDK. Read-only SDK connectivity checks do not validate physical teleoperation or the joint mapping.

## Connect and teleoperate

First place the robot in a clear workspace and put the SO-ARM101 leader and JAKA in corresponding safe poses. By default, the driver logs in and enables JAKA servo mode but does not power on or enable the robot. Prepare those states at the controller before connecting. `auto_power_on` and `auto_enable_robot` are opt-in config switches.

```bash
lerobot-teleoperate \
  --robot.type=jaka_zu7_follower \
  --robot.ip_address=10.5.5.100 \
  --robot.sdk_python=/usr/bin/python3.10 \
  --robot.sdk_dir="$HOME/Downloads/jaka_sdk/Linux/python3/x86_64-linux-gnu" \
  --robot.servo_step_num=2 \
  --robot.max_joint_velocity_deg_s=3 \
  --robot.max_joint_acceleration_deg_s2=5 \
  --teleop.type=so101_leader \
  --teleop.port=/dev/ttyACM0 \
  --teleop.id=so101_jaka_leader \
  --teleop.use_degrees=true \
  --fps=60
```

The example uses the default signs and offsets. Before running it, measure the mapping and supply `--robot.joint_signs='[s1,s2,s3,s4,s5,s6]'` and `--robot.joint_offsets_deg='[o1,o2,o3,o4,o5,o6]'` with actual numbers if needed. The target relation is `JAKA_deg = sign * leader_deg + offset_deg`. For corresponding starting poses, calculate each offset as `JAKA_start_deg - sign * leader_start_deg`. Teleoperation uses absolute targets and will move toward that mapped target even if the leader is stationary; a speed limit does not correct a wrong mapping.

For the first motion test, keep the emergency stop accessible and verify each leader joint moves the intended JAKA axis and direction with small movements. The mapping assumes both arms' six joints are in the listed order; mechanical range matching and collision avoidance are not inferred automatically. Increase the limits only after checking the mechanics, payload, workspace, and controller settings. Cameras are optional; add `--robot.cameras="{front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}"` when needed.

The example caps each joint at 3 degrees/s and 5 degrees/s². At 3 degrees/s, a 90-degree move takes roughly 30 seconds even with a steady 60 Hz control loop. After verifying the mapping and working range, increase these limits gradually, for example to `--robot.max_joint_velocity_deg_s=10 --robot.max_joint_acceleration_deg_s2=20`. Raising `fps` does not raise the joint speed limit. When the leader moves faster than the configured limit, the follower lags and continues toward the latest target after the leader stops.

If the control loop frequency changes from 60 Hz, choose `servo_step_num` so `servo_step_num * 8 ms` is close to the action interval. The JAKA Python SDK and controller are responsible for their own network timing; this driver has only been unit-tested with a mocked SDK.

### Teleoperation loop based on lerobot_mogai

[`examples/jaka/teleoperate.py`](../../../../examples/jaka/teleoperate.py) provides a dedicated loop based on the structure of [cotton365/lerobot_mogai](https://github.com/cotton365/lerobot_mogai/tree/21258a9a898678a28b59dd7c339096a87c6054c1). The reference uses `servo_j`, a joint NLF controller filter, relative startup mapping, and no synchronous follower position reads per tick. Its controller firmware and Python binding differ from the locally installed SDK: Python SDK 2.1.14 accepts `servo_j(joints_rad, 0)` with a fixed 8 ms cycle, rather than the reference's three arguments. The local script adapts to the two-argument API, uses a target rate of 125 Hz, and retains a low default speed cap of 10 deg/s. This is a structural comparison, not evidence that the reported physical two-second delay is fixed.

Stop other control programs and leave JAKA powered and enabled. Hold the leader still while connecting. Both arms' initial poses become the relative reference; there is no automatic reference-pose move and the leader need not read zero. Start from mechanically corresponding poses, check joint directions with small movements, and keep the emergency stop available.

```bash
python examples/jaka/teleoperate.py \
  --ip=10.5.5.100 \
  --sdk-python=/usr/bin/python3.10 \
  --sdk-dir=/home/dhz/Downloads/jaka_sdk/Linux/python3/x86_64-linux-gnu \
  --port=/dev/ttyACM0 \
  --leader-id=so101_jaka_leader \
  --fps=125 \
  --max-velocity-deg-s=10 \
  --max-acceleration-deg-s2=60 \
  --max-jerk-deg-s3=240
```

The NLF filter is configured before enabling servo mode, using the requested velocity, acceleration and jerk limits in degrees; the software limiter also remains active. Use `--no-configure-nlf` to preserve the controller's current filter for a comparison run. The script sends each latest leader sample once and does not queue catch-up frames. It prints measured loop frequency, leader-read and SDK-send duration, and the current difference between the mapped target and software-limited command. These durations do not measure physical response latency. A large target-command gap indicates software limiting or a leader motion faster than the speed cap; a small gap does not prove that the controller has followed the target.

The loop only reads actual JAKA joints at connection. It neither returns commanded positions as measured observations nor records a dataset. Use the standard recording path for datasets with actual robot feedback. Physical smoothness, tracking error and command-loss behavior still require hardware validation. The default driver behavior remains `servo_j_extend`, absolute mapping and unchanged controller filtering; the script opts into `servo_j`, relative mapping and NLF explicitly.

## Return to a reference pose

Stop teleoperation before resetting. `examples/jaka/reset_to_reference.py` defaults to the reference pose `[0, 90, 0, 90, 180, 0]` degrees in JAKA controller coordinates. This is a joint move, not a collision-free path planner. Verify clearance along the whole path, including the tool and payload. The robot must already be powered and enabled through the controller; the script does not release emergency stops or power/enable it automatically.

First preview the current and target angles; this does not send motion or change servo mode:

```bash
python examples/jaka/reset_to_reference.py \
  --ip=10.5.5.100 \
  --sdk-python=/usr/bin/python3.10 \
  --sdk-dir="$HOME/Downloads/jaka_sdk/Linux/python3/x86_64-linux-gnu"
```

To execute, add `--execute`. The current JAKA reset defaults are 20 degrees/s and 40 degrees/s²:

```bash
python examples/jaka/reset_to_reference.py \
  --ip=10.5.5.100 \
  --sdk-python=/usr/bin/python3.10 \
  --sdk-dir="$HOME/Downloads/jaka_sdk/Linux/python3/x86_64-linux-gnu" \
  --execute
```

Before moving, the script queries streaming servo mode using the documented read-only TCP `is_in_servomove` command on port 10001, because Python SDK 2.1.14 lacks this query. Motor enable feedback from `get_robot_state` is a different state. If already outside streaming mode, the script skips `servo_move_enable(False)`; otherwise it exits and confirms the mode changed. A failed or unknown mode query, failed exit, or unchanged streaming mode blocks the reset. SDK error `-61` means operation timeout and is not ignored.

The SDK control connection and this additional TCP connection must not overlap: on the tested controller, opening the TCP connection while logged into the SDK made later SDK commands hang, even read-only `get_tool_id`. The bridge logs out before the mode query, closes its TCP socket, and logs in again before continuing. It refuses to switch connections while a motion command or streaming control started by this bridge may be active. After querying, the reset script rechecks robot state and stopped feedback, and reads `get_tool_id` to verify the restored SDK command channel before sending a move. No motion command is automatically retried on timeout; if a command times out, its outcome may be unknown and stopping must be verified at the controller.

The script then submits an absolute `joint_move_extend` command with angles, speed and acceleration converted to radians. It polls feedback instead of blocking the SDK pipe, verifies the final angles and stopped state, and requests `motion_abort` on completion, timeout or interruption. The worker also attempts to abort its joint move on EOF or before logout. If communication fails, software cannot guarantee a stop; keep the physical emergency stop accessible. `Ctrl+C` requests a stop, and the default overall movement timeout is 120 seconds. Successful reset leaves the robot powered and enabled. Pass `--target-deg J1 J2 J3 J4 J5 J6` to specify another reference pose.
