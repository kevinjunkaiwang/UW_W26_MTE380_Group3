import serial
import time
from typing import Optional


class SerialSender:
    """Non-blocking serial line sender."""

    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.01):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None

    def open(self) -> None:
        if self.ser is None:
            self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)

    def close(self) -> None:
        if self.ser:
            self.ser.close()
            self.ser = None

    def send_line(self, line: str) -> None:
        if not self.ser:
            raise RuntimeError("Serial not opened")
        self.ser.write(line.encode("ascii"))
        # tiny pause helps on some adapters
        time.sleep(0.0005)
