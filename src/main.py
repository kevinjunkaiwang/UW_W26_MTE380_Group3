"""
Pi-side runtime controller for the line-following robot.

High-level responsibilities:
1) Capture camera frames and compute line-visibility metric X2 (L2 percentage).
2) Compute X1 from previous-loop base_v command.
3) Run fuzzy scheduler to pick (v_cap, Kp, Ki, Kd) from (X1, X2).
4) Send only base speed and PID gains back to Arduino.

Serial protocol used here:
- Command (Pi -> Arduino): C,base_v,kp,ki,kd
- Telemetry (Arduino -> Pi): T,seq,ms,ir0..ir7,left_applied,right_applied,flags
  (Reads are temporarily disabled in main loop; kept for future reintegration.)
"""

import argparse
import threading
import time
from dataclasses import dataclass
from typing import Optional

import cv2

from paper_fuzzy import FuzzyScheduler
from serial_link import SerialBridge
from vision import compute_features


def default_port() -> str:
    """Serial default for Raspberry Pi + Arduino USB connection."""
    return "/dev/ttyACM0"


@dataclass
class CameraSample:
    """Latest camera-derived features."""
    timestamp: float
    valid: int
    l2_pct: float


class CameraWorker:
    """
    Background camera reader that continuously updates latest CameraSample.

    Keeping camera I/O in a separate thread prevents capture jitter from
    blocking the control loop.
    """

    def __init__(
        self,
        camera_index: int,
        roi: float,
        thresh: int,
        row_frac: float,
        width: int,
        height: int,
        fps: float,
    ):
        self.camera_index = camera_index
        self.roi = roi
        self.thresh = thresh
        self.row_frac = row_frac
        self.width = width
        self.height = height
        self.fps = fps

        self.cap: Optional[cv2.VideoCapture] = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._latest: Optional[CameraSample] = None

    def _open_camera(self) -> cv2.VideoCapture:
        """Open camera with a Linux/Raspberry Pi first backend strategy."""
        cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            cap = cv2.VideoCapture(self.camera_index)
        if self.width > 0:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        if self.height > 0:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        if self.fps > 0:
            cap.set(cv2.CAP_PROP_FPS, self.fps)
        return cap

    def start(self) -> None:
        """Open camera and start acquisition thread."""
        self.cap = self._open_camera()
        if not self.cap.isOpened():
            raise RuntimeError(
                f"Could not open camera index {self.camera_index}. "
                "Try --camera 1 if your USB webcam is on index 1."
            )
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Stop acquisition thread and release camera resource."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def _run(self) -> None:
        """Capture loop: read frames and compute X2/L2 features."""
        assert self.cap is not None
        while not self._stop.is_set():
            ok, frame = self.cap.read()
            if not ok or frame is None:
                time.sleep(0.005)
                continue

            _, valid, l2_pct = compute_features(
                frame,
                roi_bottom_ratio=self.roi,
                thresh_val=self.thresh,
                row_occupancy_frac=self.row_frac,
            )
            now = time.monotonic()
            with self._lock:
                self._latest = CameraSample(
                    timestamp=now,
                    valid=int(valid),
                    l2_pct=l2_pct,
                )

    def latest(self) -> Optional[CameraSample]:
        """Return a snapshot copy of latest sample."""
        with self._lock:
            if self._latest is None:
                return None
            return CameraSample(
                timestamp=self._latest.timestamp,
                valid=self._latest.valid,
                l2_pct=self._latest.l2_pct,
            )


def parse_args():
    """Parse runtime options for camera, control, serial, and safety settings."""
    p = argparse.ArgumentParser(description="Pi-side controller: vision + fuzzy -> Arduino base_v + PID gains.")
    p.add_argument("--camera", type=int, default=0, help="VideoCapture index.")
    p.add_argument("--width", type=int, default=0, help="Optional camera width.")
    p.add_argument("--height", type=int, default=0, help="Optional camera height.")
    p.add_argument("--cam-fps", type=float, default=30.0, help="Camera capture target FPS.")
    p.add_argument("--roi", type=float, default=0.6, help="Bottom fraction of frame used for line sensing.")
    p.add_argument("--thresh", type=int, default=0, help="Red-mask strictness (0 = default S/V floors).")
    p.add_argument("--row-frac", type=float, default=0.02, help="Row occupancy fraction threshold.")

    p.add_argument("--port", type=str, default=default_port(), help="Serial port to Arduino.")
    p.add_argument("--baud", type=int, default=230400, help="Serial baud rate.")
    p.add_argument("--ctrl-hz", type=float, default=100.0, help="Pi control loop frequency.")

    # Reserved for future telemetry freshness gate:
    # p.add_argument("--telemetry-timeout", type=float, default=0.20, help="Max telemetry age before failsafe.")
    p.add_argument("--camera-timeout", type=float, default=0.20, help="Max camera feature age before camera invalid.")

    p.add_argument("--v-cmd", type=float, default=0.65, help="Requested forward speed command [0..1].")
    p.add_argument("--x1-alpha", type=float, default=0.8, help="EMA previous-weight for X1 proxy (0..1, higher=smoother).")
    return p.parse_args()


def main():
    """
    Main runtime loop.

    Loop steps:
    1) Refresh camera features.
    2) Compute X1/X2 inputs and run fuzzy scheduler.
    3) Select base_v and PID gains using normal/fallback logic.
    4) Send (base_v, kp, ki, kd) to Arduino.
    """
    args = parse_args()
    period = 1.0 / max(1.0, args.ctrl_hz)

    bridge = SerialBridge(args.port, args.baud)
    bridge.open()

    cam = CameraWorker(
        camera_index=args.camera,
        roi=args.roi,
        thresh=args.thresh,
        row_frac=args.row_frac,
        width=args.width,
        height=args.height,
        fps=args.cam_fps,
    )
    cam.start()

    sched = FuzzyScheduler()

    x1_f = 0.0
    prev_base_v = 0.0

    l2_pct = 0.0
    x2 = 0.0
    base_v = 0.0
    kp = 0.0
    ki = 0.0
    kd = 0.0

    next_tick = time.monotonic()
    try:
        while True:
            # 1) Telemetry reads are temporarily disabled.
            # Keep this block for future reintegration:
            #
            # for pkt in bridge.read_telemetry():
            #     last_telem = pkt

            # 2) Pull newest camera feature sample and gate by freshness.
            now = time.monotonic()
            cam_data = cam.latest()
            if cam_data is not None:
                cam_fresh = (now - cam_data.timestamp) <= args.camera_timeout
                cam_valid = cam_fresh and cam_data.valid == 1
                l2_pct = cam_data.l2_pct
            else:
                cam_fresh = False
                cam_valid = False
            # Use L2 row-coverage percentage as fuzzy X2.
            x2 = l2_pct if cam_valid else 0.0

            # 3) X1 is derived from previous-loop base_v command (0..1 -> 0..100 scale).
            x1_raw = 100.0 * abs(prev_base_v)
            x1_f = float(args.x1_alpha) * x1_f + (1.0 - float(args.x1_alpha)) * x1_raw

            if cam_valid:
                # Normal mode: use fuzzy-selected speed cap and gains.
                _, _, pid_tuple = sched.evaluate(x1_f, x2)
                v_cap, kp, ki, kd = pid_tuple
                base_v = min(args.v_cmd, v_cap)
            else:
                # Conservative fallback when camera data is stale/invalid.
                base_v = min(args.v_cmd, 0.25)
                kp, ki, kd = 0.70, 0.0, 0.10

            # 5) Send command parameters each control tick.
            bridge.send_command(base_v, kp, ki, kd)
            prev_base_v = base_v

            # 6) Maintain fixed control period.
            next_tick += period
            sleep_s = next_tick - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_tick = time.monotonic()

    finally:
        try:
            # Best-effort final stop commands before closing serial.
            for _ in range(3):
                bridge.send_command(0.0, 0.0, 0.0, 0.0)
                time.sleep(0.01)
        except Exception:
            pass
        bridge.close()
        cam.stop()


if __name__ == "__main__":
    main()
