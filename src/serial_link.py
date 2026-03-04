"""
Serial transport layer between Pi controller (`main.py`) and Arduino firmware.

Protocol:
- Pi -> Arduino command:
  C,base_v,kp,ki,kd
- Arduino -> Pi telemetry:
  T,seq,ms,ir0,ir1,ir2,ir3,ir4,ir5,ir6,ir7,left_applied,right_applied,flags

This module is intentionally small and non-blocking:
- write: one line per control tick
- read: parse all complete telemetry lines currently available
"""

import time
from dataclasses import dataclass
from typing import List, Optional

import serial


@dataclass
class Telemetry:
    """
    Parsed telemetry packet from Arduino.

    Only fields used by Pi control are retained here:
    - left_applied/right_applied: commands currently applied by Arduino
    - host_rx_time: local host monotonic timestamp when packet was parsed
    """

    left_applied: float
    right_applied: float
    host_rx_time: float


class SerialBridge:
    """
    Bidirectional serial bridge between Pi controller and Arduino motor I/O firmware.

    Typical use:
    1) open()
    2) send_command(...) every control tick
    3) read_telemetry() every control tick
    4) close() at shutdown
    """

    def __init__(self, port: str, baud: int = 230400, timeout: float = 0.0):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None
        self._rx_buf = ""

    def open(self) -> None:
        """Open serial port and reset receive buffer."""
        if self.ser is None:
            self.ser = serial.Serial(
                self.port,
                self.baud,
                timeout=self.timeout,
                write_timeout=0.0,
            )
            self._rx_buf = ""

    def close(self) -> None:
        """Close serial port and clear receive buffer."""
        if self.ser is not None:
            self.ser.close()
            self.ser = None
            self._rx_buf = ""

    def send_command(
        self,
        base_v: float,
        kp: float,
        ki: float,
        kd: float,
    ) -> None:
        """
        Send one command line to Arduino.

        base_v is expected in [0, 1] by convention. PID gains are unconstrained
        here and should be validated/clamped by firmware if needed.
        """
        if self.ser is None:
            raise RuntimeError("Serial not opened")
        line = (
            f"C,{float(base_v):.3f},{float(kp):.4f},"
            f"{float(ki):.4f},{float(kd):.4f}\n"
        )
        self.ser.write(line.encode("ascii"))

    def read_telemetry(self) -> List[Telemetry]:
        """
        Drain currently available serial bytes and parse all complete telemetry packets.

        Returns:
        - list of Telemetry objects (possibly empty)
        """
        if self.ser is None:
            raise RuntimeError("Serial not opened")

        packets: List[Telemetry] = []
        n = self.ser.in_waiting
        if n > 0:
            chunk = self.ser.read(n).decode("ascii", errors="ignore")
            self._rx_buf += chunk

        while True:
            nl = self._rx_buf.find("\n")
            if nl < 0:
                break
            raw = self._rx_buf[:nl].strip()
            self._rx_buf = self._rx_buf[nl + 1 :]
            if not raw:
                continue
            pkt = self._parse_telemetry_line(raw)
            if pkt is not None:
                packets.append(pkt)
        return packets

    @staticmethod
    def _parse_telemetry_line(line: str) -> Optional[Telemetry]:
        """Parse one raw telemetry line; return None for malformed packets."""
        # Expected from current firmware:
        # T,seq,ms,ir0,ir1,ir2,ir3,ir4,ir5,ir6,ir7,left_applied,right_applied,flags
        #
        # Pi currently uses only left_applied and right_applied.
        parts = line.split(",")
        if len(parts) != 14 or parts[0] != "T":
            return None
        try:
            left_applied = float(parts[11])
            right_applied = float(parts[12])
        except ValueError:
            return None

        return Telemetry(
            left_applied=left_applied,
            right_applied=right_applied,
            host_rx_time=time.monotonic(),
        )
