"""Serial CAN transport for the legacy Damiao USB2CAN adapter.

Frame layout follows the MIT-licensed DM_CAN.py example in
https://gitee.com/kit-miao/motor-sdk/tree/master/Python%E4%BE%8B%E7%A8%8B/u2can
Copyright (c) 2024 cmjang.
"""

import time
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class CANFrame:
    arbitration_id: int
    data: bytes


class DamiaoUSB2CAN:
    _TX_TEMPLATE = bytes(
        (
            0x55,
            0xAA,
            0x1E,
            0x03,
            0x01,
            0,
            0,
            0,
            0x0A,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0x08,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )
    )

    def __init__(self, port: str):
        try:
            import serial
        except ImportError as exc:
            raise ImportError("Damiao USB2CAN requires pyserial") from exc
        self.serial: Any = serial.Serial(port=port, baudrate=921600, timeout=0.01)
        self._pending = bytearray()

    def send(self, arbitration_id: int, data: bytes) -> None:
        if not 0 <= arbitration_id <= 0x7FF or len(data) != 8:
            raise ValueError("Damiao USB2CAN requires an 11-bit CAN ID and eight data bytes")
        frame = bytearray(self._TX_TEMPLATE)
        frame[13:15] = arbitration_id.to_bytes(2, "little")
        frame[21:29] = data
        written = self.serial.write(frame)
        if written != len(frame):
            raise ConnectionError("Damiao USB2CAN serial write was incomplete")

    def recv(self, timeout: float) -> CANFrame | None:
        deadline = time.monotonic() + timeout
        while True:
            while len(self._pending) >= 16:
                start = self._pending.find(0xAA)
                if start < 0:
                    self._pending.clear()
                    break
                if start:
                    del self._pending[:start]
                if len(self._pending) < 16:
                    break
                if self._pending[15] != 0x55:
                    del self._pending[0]
                    continue
                packet = bytes(self._pending[:16])
                del self._pending[:16]
                # Only 0x11 is a CAN receive packet in the official u2can example.
                if packet[1] == 0x11:
                    can_id = int.from_bytes(packet[3:7], "little")
                    if can_id <= 0x7FF:
                        return CANFrame(can_id, packet[7:15])
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            self.serial.timeout = min(0.01, remaining)
            # Block for one byte only; drain bytes already available without
            # waiting for an arbitrary 64-byte buffer to fill.
            self._pending.extend(self.serial.read(min(256, self.serial.in_waiting) or 1))

    def shutdown(self) -> None:
        self.serial.close()
