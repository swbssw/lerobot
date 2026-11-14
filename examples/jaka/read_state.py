"""Read JAKA joint positions through the SDK subprocess without enabling motion."""

import argparse
import math

from lerobot.robots.jaka_zu7_follower.sdk_bridge import JakaSDKBridge


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip", required=True, help="JAKA controller IP address")
    parser.add_argument("--sdk-python", required=True, help="SDK-compatible Python executable")
    parser.add_argument("--sdk-dir", required=True, help="Directory containing jkrc.so and libjakaAPI.so")
    args = parser.parse_args()

    with JakaSDKBridge(args.ip, args.sdk_python, args.sdk_dir) as sdk:
        result = sdk.login()
        if result[0] != 0:
            raise RuntimeError(f"JAKA login failed: {result}")
        try:
            result = sdk.get_joint_position()
            if result[0] != 0:
                raise RuntimeError(f"JAKA get_joint_position failed: {result}")
            print("J1-J6 (degrees):", [round(math.degrees(value), 3) for value in result[1]])
        finally:
            result = sdk.logout()
            if result[0] != 0:
                raise RuntimeError(f"JAKA logout failed: {result}")


if __name__ == "__main__":
    main()
