"""
Simulation/runtime loop for offline testing:

1) Capture camera frames.
2) Compute vision features (lk_norm, L2, validity).
3) Run fuzzy scheduler with the same control logic as main.py.
4) Print PID command values to terminal over time.

No serial communication is performed.
"""

import argparse
import threading
import time
from dataclasses import dataclass
from typing import Optional

from paper_fuzzy import FuzzyScheduler
from vision import compute_features

import cv2
import numpy as np


@dataclass
class CameraSample:
    """Latest camera-derived features."""

    timestamp: float
    lk: float
    valid: int
    l2_pct: float
    frame: Optional[np.ndarray]


class CameraWorker:
    """Background camera reader that continuously updates latest CameraSample."""

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
        """Open camera with a cross-platform-friendly backend strategy."""
        # On Windows, V4L2 can block/unavailable; try DirectShow first.
        cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)
        if not cap.isOpened():
            cap.release()
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
                "Try different --camera values (0, 1, 2)."
            )
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Stop acquisition and release camera resource."""
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

            lk, valid, l2_pct = compute_features(
                frame,
                roi_bottom_ratio=self.roi,
                thresh_val=self.thresh,
                row_occupancy_frac=self.row_frac,
            )
            now = time.monotonic()
            with self._lock:
                self._latest = CameraSample(
                    timestamp=now,
                    lk=lk,
                    valid=int(valid),
                    l2_pct=l2_pct,
                    frame=frame.copy(),
                )

    def latest(self) -> Optional[CameraSample]:
        """Return a snapshot copy of the latest sample."""
        with self._lock:
            if self._latest is None:
                return None
            return CameraSample(
                timestamp=self._latest.timestamp,
                lk=self._latest.lk,
                valid=self._latest.valid,
                l2_pct=self._latest.l2_pct,
                frame=(None if self._latest.frame is None else self._latest.frame.copy()),
            )


def parse_args():
    """Parse simulation options for camera, control, and print cadence."""
    p = argparse.ArgumentParser(
        description="Vision + fuzzy control simulation: print PID outputs in terminal."
    )
    p.add_argument("--camera", type=int, default=0, help="VideoCapture index.")
    p.add_argument("--width", type=int, default=0, help="Optional camera width.")
    p.add_argument("--height", type=int, default=0, help="Optional camera height.")
    p.add_argument("--cam-fps", type=float, default=30.0, help="Camera capture target FPS.")
    p.add_argument("--roi", type=float, default=0.6, help="Bottom fraction of frame used for line sensing.")
    p.add_argument("--thresh", type=int, default=0, help="Line threshold (0 = auto Otsu).")
    p.add_argument(
        "--row-frac",
        type=float,
        default=0.02,
        help="Row occupancy fraction threshold.",
    )
    p.add_argument("--ctrl-hz", type=float, default=10.0, help="Control loop frequency.")
    p.add_argument("--print-hz", type=float, default=10.0, help="How often to print PID values.")
    p.add_argument("--camera-timeout", type=float, default=0.20, help="Max camera feature age before invalid.")
    p.add_argument("--v-cmd", type=float, default=0.80, help="Requested forward speed command [0..1].")
    p.add_argument("--x1-alpha", type=float, default=0.8, help="EMA smoothing factor for X1 proxy (0..1).")
    return p.parse_args()


def main():
    """Run control loop and print PID output values from fuzzy scheduler."""
    args = parse_args()
    period = 1.0 / max(1.0, args.ctrl_hz)
    print_period = 1.0 / max(1.0, args.print_hz)

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
    lk_norm = 0.0
    x2 = 0.0
    base_v = 0.0
    kp = 0.0
    ki = 0.0
    kd = 0.0
    label = "LC"
    x_star = 0.0
    label_disp = "NO_LINE"
    x_star_disp = 0.0

    next_tick = time.monotonic()
    next_print = time.monotonic()

    try:
        while True:
            now = time.monotonic()
            cam_data = cam.latest()

            if cam_data is not None:
                cam_fresh = (now - cam_data.timestamp) <= args.camera_timeout
                cam_valid = cam_fresh and cam_data.valid == 1
                l2_pct = cam_data.l2_pct
                lk_norm = cam_data.lk
                frame = cam_data.frame
            else:
                cam_fresh = False
                cam_valid = False
                lk_norm = 0.0
                frame = None
            # Use look-ahead depth as X2 (paper-aligned distance proxy), not row-coverage percent.
            x2 = 100.0 * lk_norm if cam_valid else 0.0

            # X1 is derived from previous-loop base_v command (0..1 -> 0..100 scale).
            x1_raw = 100.0 * abs(prev_base_v)
            x1_f = float(args.x1_alpha) * x1_f + (1.0 - float(args.x1_alpha)) * x1_raw

            x_star, label, pid_tuple = sched.evaluate(x1_f, x2)
            v_cap, kp, ki, kd = pid_tuple

            if cam_valid:
                base_v = min(args.v_cmd, v_cap)
                label_disp = label
                x_star_disp = x_star
            else:
                base_v = min(args.v_cmd, 0.25)
                kp, ki, kd = 0.70, 0.0, 0.10
                label_disp = "NO_LINE"
                x_star_disp = 0.0

            prev_base_v = base_v

            now_print = time.monotonic()
            if now_print >= next_print:
                print(
                    f"lk={lk_norm:5.3f} l2={l2_pct:6.2f}% x1={x1_f:5.2f} x2={x2:6.2f} x*={x_star_disp:6.2f} out_label={label_disp} "
                    f"base_v={base_v:0.3f} kp={kp:0.4f} ki={ki:0.4f} kd={kd:0.4f}"
                )
                next_print = now_print + print_period

            if frame is not None:
                display = frame.copy()
                h, w = display.shape[:2]
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.6
                thickness = 1
                margin = 8

                y0 = int(h * (1.0 - args.roi))
                y0 = int(max(0, min(h - 1, y0)))
                cv2.line(display, (0, y0), (w - 1, y0), (0, 255, 255), 2)

                lines = [
                    f"out_label={label_disp}",
                    f"x*={x_star_disp:5.2f}",
                    f"lk={lk_norm:5.3f}",
                    f"l2={l2_pct:5.1f}%",
                    f"x1={x1_f:5.2f}",
                    f"x2={x2:5.2f}",
                    f"base_v={base_v:0.3f}",
                    f"kp={kp:0.4f}",
                    f"ki={ki:0.4f}",
                    f"kd={kd:0.4f}",
                ]
                for idx, line in enumerate(lines):
                    y = margin + (idx + 1) * 24
                    cv2.putText(
                        display,
                        line,
                        (margin, y),
                        font,
                        font_scale,
                        (0, 255, 0),
                        thickness,
                        cv2.LINE_AA,
                    )

                cv2.imshow("vision+sim", display)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            next_tick += period
            sleep_s = next_tick - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_tick = time.monotonic()

    finally:
        cam.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
