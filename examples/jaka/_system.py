"""交互式选择运行系统,并为 JAKA 脚本补齐 OS 相关的默认参数。

``teleoperate.py`` 和 ``reset_to_reference.py`` 都需要三个随系统而变的参数:
SDK 解释器(sdk_python)、jkrc SDK 目录(sdk_dir)、主臂串口(port)。
启动时先问一次运行系统(默认自动检测),再返回对应的默认值;
命令行显式传入的值始终优先于这些默认值。
"""

import os
import sys

SYSTEM_DEFAULTS = {
    "windows": {
        "sdk_python": r"C:\Users\Administrator\miniconda3\envs\jaka\python.exe",
        "sdk_dir": r"E:\A_practice3\jaka_sdk\Windows\python3\x64",
        "port": "COM8",
    },
    "linux": {
        "sdk_python": "/usr/bin/python3.10",
        "sdk_dir": "/home/dhz/Downloads/jaka_sdk/Linux/python3/x86_64-linux-gnu",
        "port": "/dev/ttyACM0",
    },
}


def detect_system() -> str:
    return "windows" if os.name == "nt" else "linux"


def resolve_system() -> str:
    detected = detect_system()
    if not sys.stdin.isatty():
        print(f"[launch] 非交互环境,自动选择系统: {detected}")
        return detected
    answer = input(f"运行系统? (windows / linux) [默认 {detected}]: ").strip().lower()
    if not answer:
        return detected
    if answer.startswith("w"):
        return "windows"
    if answer.startswith("l"):
        return "linux"
    print(f"[launch] 无法识别 '{answer}',使用检测值 {detected}")
    return detected


def choose() -> dict[str, str]:
    """返回所选系统的默认参数字典 {sdk_python, sdk_dir, port}。"""
    system = resolve_system()
    defaults = dict(SYSTEM_DEFAULTS[system])
    print(
        f"[launch] 系统={system} | sdk_python={defaults['sdk_python']} | "
        f"sdk_dir={defaults['sdk_dir']} | port={defaults['port']}"
    )
    return defaults
