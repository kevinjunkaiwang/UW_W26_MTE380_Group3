"""
Raspberry Pi line-following vision node that sends driving commands to Arduino.

Command protocol (newline-terminated ASCII):
  F -> forward
  L -> turn left
  R -> turn right
  S -> stop
"""

import argparse
import time

import cv2
import numpy as np
import serial


def parse_args():
    parser = argparse.ArgumentParser(
        description="Line-follow camera pipeline with serial commands to Arduino."
    )
    parser.add_argument("--camera-index", type=int, default=1, help="VideoCapture index.")
    parser.add_argument("--width", type=int, default=640, help="Requested capture width.")
    parser.add_argument("--height", type=int, default=480, help="Requested capture height.")
    parser.add_argument("--cam-fps", type=float, default=30.0, help="Requested camera FPS.")
    parser.add_argument("--threshold", type=int, default=100, help="Binary threshold value.")
    parser.add_argument(
        "--roi-bottom-ratio",
        type=float,
        default=0.45,
        help="Bottom fraction of frame used for line detection.",
    )
    parser.add_argument(
        "--left-frac",
        type=float,
        default=0.35,
        help="Left turn threshold as frame-width fraction.",
    )
    parser.add_argument(
        "--right-frac",
        type=float,
        default=0.65,
        help="Right turn threshold as frame-width fraction.",
    )
    parser.add_argument(
        "--serial-port",
        type=str,
        default="/dev/ttyACM0",
        help="Arduino serial device path.",
    )
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate.")
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


def write_cmd(ser, cmd):
    ser.write((cmd + "\n").encode("ascii"))


def main():
    args = parse_args()

    if not (0.0 < args.roi_bottom_ratio <= 1.0):
        raise ValueError("--roi-bottom-ratio must be in (0, 1].")
    if not (0.0 < args.left_frac < 1.0 and 0.0 < args.right_frac < 1.0):
        raise ValueError("--left-frac and --right-frac must be in (0, 1).")
    if args.left_frac >= args.right_frac:
        raise ValueError("--left-frac must be less than --right-frac.")

    cap = open_camera(args.camera_index, args.width, args.height, args.cam_fps)
    if not cap.isOpened():
        raise RuntimeError(
            "Could not open camera index {}. Try another --camera-index.".format(
                args.camera_index
            )
        )

    ser = serial.Serial(args.serial_port, args.baud, timeout=0.05)
    time.sleep(1.5)
    write_cmd(ser, "S")

    if args.show_window:
        cv2.namedWindow("line-follow", cv2.WINDOW_NORMAL)

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
            label = "NO LINE"
            if contours:
                c = max(contours, key=cv2.contourArea)
                moments = cv2.moments(c)
                if moments["m00"] != 0:
                    cx = int(moments["m10"] / moments["m00"])
                    cy = int(moments["m01"] / moments["m00"])

                    cmd = classify_state(cx, left_bound, right_bound)
                    line_missing_streak = 0

                    if cmd == "F":
                        label = "FORWARD"
                    elif cmd == "L":
                        label = "TURN LEFT"
                    else:
                        label = "TURN RIGHT"

                    if args.show_window:
                        cv2.drawContours(roi, [c], -1, (0, 255, 0), 2)
                        cv2.circle(roi, (cx, cy), 4, (0, 0, 255), -1)
            else:
                line_missing_streak += 1
                if line_missing_streak < 3 and last_cmd in ("L", "R"):
                    cmd = last_cmd
                    label = "SEARCH {}".format("LEFT" if last_cmd == "L" else "RIGHT")

            now = time.monotonic()
            if cmd != last_cmd or (now - last_send_time) >= min_interval:
                write_cmd(ser, cmd)
                last_cmd = cmd
                last_send_time = now

            if args.show_window:
                cv2.line(image, (left_bound, 0), (left_bound, frame_h - 1), (255, 0, 0), 1)
                cv2.line(
                    image, (right_bound, 0), (right_bound, frame_h - 1), (255, 0, 0), 1
                )
                cv2.line(image, (0, roi_y0), (frame_w - 1, roi_y0), (255, 255, 0), 1)
                cv2.putText(
                    image,
                    "CMD: {}".format(label),
                    (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.imshow("line-follow", image)

                # If user closes the window (X button), stop robot and exit.
                if cv2.getWindowProperty("line-follow", cv2.WND_PROP_VISIBLE) < 1:
                    write_cmd(ser, "S")
                    break

                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    write_cmd(ser, "S")
                    break
    finally:
        try:
            write_cmd(ser, "S")
        except Exception:
            pass
        ser.close()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
