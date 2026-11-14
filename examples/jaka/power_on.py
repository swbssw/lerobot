"""Power on and enable the JAKA Zu 7 (队友流程的第 1 步:上电使能)。"""

import argparse
import math
import time
from typing import Any

from lerobot.robots.jaka_zu7_follower.sdk_bridge import JakaSDKBridge

if __package__:
    from . import _system
else:
    import _system


def check_result(result: Any, operation: str) -> Any:
    if not isinstance(result, (list, tuple)) or not result or result[0] != 0:
        raise RuntimeError(f"JAKA {operation} failed: {result}")
    return result


def power_on_and_enable(sdk: JakaSDKBridge, power_on_delay_s: float = 8.0) -> None:
    """Power and enable only if needed, checking feedback before each step."""
    if not math.isfinite(power_on_delay_s) or power_on_delay_s < 0:
        raise ValueError("Power-on delay must be finite and non-negative")
    state = check_result(sdk.get_robot_state(), "get_robot_state")[1]
    if state[0]:
        raise RuntimeError("请先松开急停按钮再上电")
    if not state[1]:
        check_result(sdk.power_on(), "power_on")
        print(f"已发送上电指令,等待 {power_on_delay_s}s ...")
        time.sleep(power_on_delay_s)
    state = check_result(sdk.get_robot_state(), "get_robot_state")[1]
    if state[0] or not state[1]:
        raise RuntimeError(f"JAKA 未完成上电或急停未释放: {state}")
    if not state[2]:
        check_result(sdk.enable_robot(), "enable_robot")
    state = check_result(sdk.get_robot_state(), "get_robot_state")[1]
    if state[0] or not state[1] or not state[2]:
        raise RuntimeError(f"JAKA 上电使能未成功: {state}")
    print(f"最终状态 -> 急停: {state[0]}; 上电: {state[1]}; 使能: {state[2]}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip", default="10.5.5.100")
    parser.add_argument("--sdk-python", default=None, help="SDK 解释器(默认按所选系统填入)")
    parser.add_argument("--sdk-dir", default=None, help="jkrc SDK 目录(默认按所选系统填入)")
    parser.add_argument("--power-on-delay-s", type=float, default=8.0)
    args = parser.parse_args()

    defaults = _system.choose()
    args.sdk_python = args.sdk_python or defaults["sdk_python"]
    args.sdk_dir = args.sdk_dir or defaults["sdk_dir"]

    with JakaSDKBridge(args.ip, args.sdk_python, args.sdk_dir, timeout_s=15.0) as sdk:
        check_result(sdk.login(), "login")
        power_on_and_enable(sdk, args.power_on_delay_s)


if __name__ == "__main__":
    main()
