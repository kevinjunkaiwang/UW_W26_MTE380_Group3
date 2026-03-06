"""
Raspberry Pi line-following vision node that sends driving commands to Arduino.

Command protocol (single-byte ASCII over I2C):
  F -> forward
  L -> turn left
  R -> turn right
  S -> stop
"""

import argparse
import time

import cv2
import numpy as np
from smbus2 import SMBus


def parse_args():
    parser = argparse.ArgumentParser(
        description="Line-follow camera pipeline with I2C commands to Arduino."
    )
    parser.add_argument("--camera-index", type=int, default=1, help="VideoCapture index.")
    parser.add_argument("--width", type=int, default=1280, help="Requested capture width.")
    parser.add_argument("--height", type=int, default=720, help="Requested capture height.")
    parser.add_argument("--cam-fps", type=float, default=30.0, help="Requested camera FPS.")
    parser.add_argument("--threshold", type=int, default=100, help="Binary threshold value.")
    parser.add_argument(
        "--roi-bottom-ratio",
        type=float,
        default=1.0,
        help="Bottom fraction of frame used for line detection.",
    )
    parser.add_argument(
        "--left-frac",
        type=float,
        default=(75.0 / 192.0),
        help="Left turn threshold as frame-width fraction.",
    )
    parser.add_argument(
        "--right-frac",
        type=float,
        default=(115.0 / 192.0),
        help="Right turn threshold as frame-width fraction.",
    )
    parser.add_argument(
        "--i2c-bus",
        type=int,
        default=1,
        help="Linux I2C bus number, e.g. SMBus(1) for /dev/i2c-1.",
    )
    parser.add_argument(
        "--i2c-address",
        type=lambda value: int(value, 0),
        default=0x08,
        help="Arduino I2C slave address. Accepts decimal or hex, e.g. 0x08.",
    )
    parser.add_argument(
        "--command-rate-hz",
        type=float,
        default=20.0,
        help="Minimum command send rate for keepalive.",
    )
    parser.add_argument(
        "--show-window",
        action="store_true",
        help="Display OpenCV debug window.",
    )
    return parser.parse_args()


def open_camera(index, width, height, fps):
    backend_candidates = [cv2.CAP_DSHOW, cv2.CAP_V4L2, None]
    for backend in backend_candidates:
        cap = cv2.VideoCapture(index) if backend is None else cv2.VideoCapture(index, backend)
        if not cap.isOpened():
            cap.release()
            continue

        if width > 0:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height > 0:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if fps > 0:
            cap.set(cv2.CAP_PROP_FPS, fps)
        return cap
    return cv2.VideoCapture(index)


def unpack_contours(result):
    if len(result) == 2:
        contours, hierarchy = result
    else:
        _, contours, hierarchy = result
    return contours, hierarchy


def classify_state(cx, left_bound, right_bound):
    if cx <= left_bound:
        return "L"
    if cx >= right_bound:
        return "R"
    return "F"


def motor_speeds_from_command(cmd):
    # Match Arduino logic: F/L/R/S map directly to left/right motor ON/OFF.
    full = 255
    if cmd == "F":
        return full, full, "FORWARD"
    if cmd == "L":
        return 0, full, "TURN LEFT"
    if cmd == "R":
        return full, 0, "TURN RIGHT"
    return 0, 0, "STOP"


def write_cmd(bus, addr, cmd):
    bus.write_byte(addr, ord(cmd))


def main():
    args = parse_args()

    if not (0.0 < args.roi_bottom_ratio <= 1.0):
        raise ValueError("--roi-bottom-ratio must be in (0, 1].")
    if not (0.0 < args.left_frac < 1.0 and 0.0 < args.right_frac < 1.0):
        raise ValueError("--left-frac and --right-frac must be in (0, 1).")
    if args.left_frac >= args.right_frac:
        raise ValueError("--left-frac must be less than --right-frac.")
    if not (0x03 <= args.i2c_address <= 0x77):
        raise ValueError("--i2c-address must be in the valid 7-bit range 0x03-0x77.")

    cap = open_camera(args.camera_index, args.width, args.height, args.cam_fps)
    if not cap.isOpened():
        raise RuntimeError(
            "Could not open camera index {}. Try another --camera-index.".format(
                args.camera_index
            )
        )

    bus = SMBus(args.i2c_bus)
    time.sleep(0.1)
    write_cmd(bus, args.i2c_address, "S")

    if args.show_window:
        cv2.namedWindow("img", cv2.WINDOW_NORMAL)
        actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if actual_width <= 0:
            actual_width = args.width if args.width > 0 else 960
        if actual_height <= 0:
            actual_height = args.height if args.height > 0 else 540
        cv2.resizeWindow("img", actual_width, actual_height)

    kernel = np.ones((3, 3), dtype=np.uint8)
    min_interval = 1.0 / max(1.0, args.command_rate_hz)
    last_cmd = None
    last_send_time = 0.0
    line_missing_streak = 0

    try:
        while True:
            ok, image = cap.read()
            if not ok or image is None:
                continue

            frame_h, frame_w = image.shape[:2]
            roi_y0 = int((1.0 - args.roi_bottom_ratio) * frame_h)
            roi = image[roi_y0:frame_h, :]

            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (5, 5), 0)
            _, thresh = cv2.threshold(blur, args.threshold, 255, cv2.THRESH_BINARY_INV)

            mask = cv2.erode(thresh, kernel, iterations=2)
            mask = cv2.dilate(mask, kernel, iterations=2)

            contours, _ = unpack_contours(
                cv2.findContours(mask.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
            )

            left_bound = int(args.left_frac * frame_w)
            right_bound = int(args.right_frac * frame_w)

            cmd = "S"
            if contours:
                c = max(contours, key=cv2.contourArea)
                moments = cv2.moments(c)
                if moments["m00"] != 0:
                    cx = int(moments["m10"] / moments["m00"])
                    cy = int(moments["m01"] / moments["m00"])

                    cmd = classify_state(cx, left_bound, right_bound)
                    line_missing_streak = 0

                    if args.show_window:
                        cv2.drawContours(roi, [c], -1, (0, 255, 0), 1)
                        cv2.circle(roi, (cx, cy), 3, (0, 0, 255), -1)
            else:
                line_missing_streak += 1
                if line_missing_streak < 3 and last_cmd in ("L", "R"):
                    cmd = last_cmd

            now = time.monotonic()
            if cmd != last_cmd or (now - last_send_time) >= min_interval:
                write_cmd(bus, args.i2c_address, cmd)
                last_cmd = cmd
                last_send_time = now

            left_speed, right_speed, state_label = motor_speeds_from_command(cmd)
            left_pct = int(round((left_speed / 255.0) * 100))
            right_pct = int(round((right_speed / 255.0) * 100))

            if args.show_window:
                cv2.line(image, (left_bound, 0), (left_bound, frame_h - 1), (255, 0, 0), 1)
                cv2.line(
                    image, (right_bound, 0), (right_bound, frame_h - 1), (255, 0, 0), 1
                )
                cv2.putText(
                    image,
                    "CMD: {} ({})".format(cmd, state_label),
                    (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    image,
                    "L: {} ({}%)  R: {} ({}%)".format(
                        left_speed, left_pct, right_speed, right_pct
                    ),
                    (8, 52),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )
                cv2.imshow("img", image)

                # If user closes the window (X button), stop robot and exit.
                if cv2.getWindowProperty("img", cv2.WND_PROP_VISIBLE) < 1:
                    write_cmd(bus, args.i2c_address, "S")
                    break

                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    write_cmd(bus, args.i2c_address, "S")
                    break
    finally:
        try:
            write_cmd(bus, args.i2c_address, "S")
        except Exception:
            pass
        bus.close()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
