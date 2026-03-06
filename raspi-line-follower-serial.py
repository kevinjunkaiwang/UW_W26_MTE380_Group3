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
    parser.add_argument(
        "mode",
        nargs="?",
        choices=["test"],
        help="Set to 'test' to bypass vision and type F/L/R/S commands manually.",
    )
    parser.add_argument("--camera-index", type=int, default=1, help="VideoCapture index.")
    parser.add_argument("--width", type=int, default=1280, help="Requested capture width.")
    parser.add_argument("--height", type=int, default=720, help="Requested capture height.")
    parser.add_argument("--cam-fps", type=float, default=30.0, help="Requested camera FPS.")
    parser.add_argument(
        "--threshold",
        type=int,
        default=100,
        help="Binary threshold value used when --threshold-mode=fixed.",
    )
    parser.add_argument(
        "--threshold-mode",
        type=str,
        choices=["otsu", "adaptive", "fixed"],
        default="otsu",
        help="Threshold strategy (default: otsu).",
    )
    parser.add_argument(
        "--adaptive-block-size",
        type=int,
        default=31,
        help="Adaptive threshold neighborhood size (odd integer).",
    )
    parser.add_argument(
        "--adaptive-c",
        type=int,
        default=5,
        help="Adaptive threshold subtraction constant.",
    )
    parser.add_argument(
        "--min-area",
        type=float,
        default=120.0,
        help="Minimum contour area accepted inside a scan band.",
    )
    parser.add_argument(
        "--roi-bottom-ratio",
        type=float,
        default=1.0,
        help="Bottom fraction of frame used as the search region for the scan bands.",
    )
    parser.add_argument(
        "--near-band-frac",
        type=float,
        default=0.20,
        help="Height fraction of the near (bottom) scan band within the search region.",
    )
    parser.add_argument(
        "--lookahead-band-frac",
        type=float,
        default=0.14,
        help="Height fraction of the upper lookahead scan band within the search region.",
    )
    parser.add_argument(
        "--lookahead-offset-frac",
        type=float,
        default=0.34,
        help="Vertical offset of lookahead band from the bottom of the search region.",
    )
    parser.add_argument(
        "--lookahead-weight",
        type=float,
        default=0.35,
        help="Blending weight of lookahead center in fused center [0..1].",
    )
    parser.add_argument(
        "--ema-alpha",
        type=float,
        default=0.75,
        help="EMA previous-weight for center smoothing [0..1).",
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


def command_from_centroid(cx, left_bound, right_bound):
    if cx >= right_bound:
        return "R"
    if cx <= left_bound:
        return "L"
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


def threshold_mask(gray_blur, mode, fixed_thresh, adaptive_block_size, adaptive_c):
    if mode == "fixed":
        _, thresh = cv2.threshold(gray_blur, fixed_thresh, 255, cv2.THRESH_BINARY_INV)
        return thresh
    if mode == "adaptive":
        block_size = max(3, int(adaptive_block_size))
        if block_size % 2 == 0:
            block_size += 1
        return cv2.adaptiveThreshold(
            gray_blur,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            block_size,
            adaptive_c,
        )

    _, thresh = cv2.threshold(
        gray_blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
    )
    return thresh


def find_band_center(mask, y0, y1, min_area):
    if y1 <= y0:
        return None, None, 0.0

    band = mask[y0:y1, :]
    contours, _ = unpack_contours(
        cv2.findContours(band.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    )
    if not contours:
        return None, None, 0.0

    filtered = [c for c in contours if cv2.contourArea(c) >= min_area]
    if not filtered:
        return None, None, 0.0

    contour = max(filtered, key=cv2.contourArea)
    moments = cv2.moments(contour)
    if moments["m00"] == 0:
        return None, None, 0.0

    cx = int(moments["m10"] / moments["m00"])
    cy = y0 + int(moments["m01"] / moments["m00"])
    return cx, cy, float(cv2.contourArea(contour))


def update_ema(prev, measurement, alpha):
    if measurement is None:
        return prev
    if prev is None:
        return float(measurement)
    return alpha * prev + (1.0 - alpha) * float(measurement)


def run_test_mode(args):
    bus = None
    try:
        bus = SMBus(args.i2c_bus)
        time.sleep(0.1)
        write_cmd(bus, args.i2c_address, "S")
        print("Test mode: type F, L, R, or S to send a command. Type Q to quit.")

        while True:
            try:
                raw = input("cmd> ")
            except (EOFError, KeyboardInterrupt):
                print()
                break

            cmd = raw.strip().upper()
            if not cmd:
                continue
            if cmd in ("Q", "QUIT", "EXIT"):
                break
            if cmd not in ("F", "L", "R", "S"):
                print("Invalid command. Use F, L, R, S, or Q.")
                continue

            write_cmd(bus, args.i2c_address, cmd)
            _, _, state_label = motor_speeds_from_command(cmd)
            print("Sent {} ({})".format(cmd, state_label))
    finally:
        try:
            if bus is not None:
                write_cmd(bus, args.i2c_address, "S")
        except Exception:
            pass
        if bus is not None:
            bus.close()


def main():
    args = parse_args()

    if not (0x03 <= args.i2c_address <= 0x77):
        raise ValueError("--i2c-address must be in the valid 7-bit range 0x03-0x77.")

    if args.mode == "test":
        run_test_mode(args)
        return

    if not (0.0 < args.roi_bottom_ratio <= 1.0):
        raise ValueError("--roi-bottom-ratio must be in (0, 1].")
    if args.min_area < 0.0:
        raise ValueError("--min-area must be >= 0.")
    if args.near_band_frac <= 0 or args.lookahead_band_frac <= 0:
        raise ValueError("--near-band-frac and --lookahead-band-frac must be > 0.")
    if args.lookahead_offset_frac <= 0 or args.lookahead_offset_frac >= 1:
        raise ValueError("--lookahead-offset-frac must be in (0, 1).")
    if not (0.0 <= args.lookahead_weight <= 1.0):
        raise ValueError("--lookahead-weight must be in [0, 1].")
    if not (0.0 <= args.ema_alpha < 1.0):
        raise ValueError("--ema-alpha must be in [0, 1).")
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

    bus = None
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
    last_sent_cmd = None
    last_send_time = 0.0
    near_cx_ema = None
    look_cx_ema = None
    miss_streak = 0

    try:
        bus = SMBus(args.i2c_bus)
        time.sleep(0.1)
        write_cmd(bus, args.i2c_address, "S")

        while True:
            ok, image = cap.read()
            if not ok or image is None:
                continue

            frame_h, frame_w = image.shape[:2]
            left_bound = int(args.left_frac * frame_w)
            right_bound = int(args.right_frac * frame_w)

            search_y0 = int((1.0 - args.roi_bottom_ratio) * frame_h)
            search_h = max(1, frame_h - search_y0)

            near_band_h = max(10, int(args.near_band_frac * search_h))
            near_y1 = frame_h
            near_y0 = max(search_y0, near_y1 - near_band_h)

            look_band_h = max(10, int(args.lookahead_band_frac * search_h))
            look_y1 = frame_h - int(args.lookahead_offset_frac * search_h)
            look_y1 = min(max(search_y0 + look_band_h, look_y1), frame_h)
            look_y0 = max(search_y0, look_y1 - look_band_h)
            if look_y1 > near_y0:
                look_y1 = near_y0
                look_y0 = max(search_y0, look_y1 - look_band_h)

            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (5, 5), 0)
            thresh = threshold_mask(
                blur,
                args.threshold_mode,
                args.threshold,
                args.adaptive_block_size,
                args.adaptive_c,
            )
            mask = cv2.erode(thresh, kernel, iterations=2)
            mask = cv2.dilate(mask, kernel, iterations=2)

            near_cx, near_cy, near_area = find_band_center(
                mask, near_y0, near_y1, args.min_area
            )
            look_cx, look_cy, look_area = find_band_center(
                mask, look_y0, look_y1, args.min_area
            )

            near_cx_ema = update_ema(near_cx_ema, near_cx, args.ema_alpha)
            look_cx_ema = update_ema(look_cx_ema, look_cx, args.ema_alpha)
            cmd = "S"
            fused_cx = None
            near_for_fuse = near_cx_ema if near_cx is not None else None
            look_for_fuse = look_cx_ema if look_cx is not None else None
            if near_for_fuse is None and look_for_fuse is None:
                miss_streak += 1
                if miss_streak >= 5:
                    near_cx_ema = None
                    look_cx_ema = None
            else:
                miss_streak = 0

            if near_for_fuse is not None and look_for_fuse is not None:
                fused_cx = int(
                    round(
                        (1.0 - args.lookahead_weight) * near_for_fuse
                        + args.lookahead_weight * look_for_fuse
                    )
                )
            elif near_for_fuse is not None:
                fused_cx = int(round(near_for_fuse))
            elif look_for_fuse is not None:
                fused_cx = int(round(look_for_fuse))

            if fused_cx is not None:
                cmd = command_from_centroid(fused_cx, left_bound, right_bound)

            now = time.monotonic()
            if cmd != last_sent_cmd or (now - last_send_time) >= min_interval:
                try:
                    write_cmd(bus, args.i2c_address, cmd)
                except OSError as exc:
                    raise RuntimeError(
                        "I2C write failed on bus {} to address {}.".format(
                            args.i2c_bus, hex(args.i2c_address)
                        )
                    ) from exc
                last_sent_cmd = cmd
                last_send_time = now

            left_speed, right_speed, state_label = motor_speeds_from_command(cmd)
            left_pct = int(round((left_speed / 255.0) * 100))
            right_pct = int(round((right_speed / 255.0) * 100))

            if args.show_window:
                cv2.line(image, (left_bound, 0), (left_bound, frame_h - 1), (255, 0, 0), 1)
                cv2.line(
                    image, (right_bound, 0), (right_bound, frame_h - 1), (255, 0, 0), 1
                )
                if search_y0 > 0:
                    cv2.line(image, (0, search_y0), (frame_w - 1, search_y0), (80, 80, 80), 1)
                cv2.rectangle(
                    image, (0, near_y0), (frame_w - 1, near_y1 - 1), (0, 180, 255), 1
                )
                cv2.rectangle(
                    image, (0, look_y0), (frame_w - 1, look_y1 - 1), (255, 180, 0), 1
                )
                if near_cx is not None and near_cy is not None:
                    cv2.circle(image, (near_cx, near_cy), 4, (0, 165, 255), -1)
                if look_cx is not None and look_cy is not None:
                    cv2.circle(image, (look_cx, look_cy), 4, (255, 165, 0), -1)
                if fused_cx is not None:
                    cv2.line(
                        image,
                        (fused_cx, 0),
                        (fused_cx, frame_h - 1),
                        (255, 255, 255),
                        1,
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
                cv2.putText(
                    image,
                    "near_x={} area={:.0f}  look_x={} area={:.0f}  mode={}".format(
                        "-" if near_cx is None else near_cx,
                        near_area,
                        "-" if look_cx is None else look_cx,
                        look_area,
                        args.threshold_mode,
                    ),
                    (8, 80),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (220, 220, 220),
                    1,
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
            if bus is not None:
                write_cmd(bus, args.i2c_address, "S")
        except Exception:
            pass
        if bus is not None:
            bus.close()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
