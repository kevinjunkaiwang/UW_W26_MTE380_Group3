"""
Raspberry Pi line-following vision node that sends driving commands to Arduino.

Command protocol (ASCII text over raw I2C writes):
  left <0..255>  -> left motor PWM
  right <0..255> -> right motor PWM
  D              -> green marker detected
  B              -> blue marker detected
"""

import argparse
import time

import cv2
import numpy as np
from smbus2 import SMBus, i2c_msg


GREEN_LOWER_HSV = np.array([35, 70, 70], dtype=np.uint8)
GREEN_UPPER_HSV = np.array([90, 255, 255], dtype=np.uint8)
BLUE_LOWER_HSV = np.array([95, 70, 70], dtype=np.uint8)
BLUE_UPPER_HSV = np.array([130, 255, 255], dtype=np.uint8)
GREEN_MORPH_KERNEL = np.ones((3, 3), dtype=np.uint8)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Line-follow camera pipeline with I2C motor-speed messages to Arduino."
    )
    parser.add_argument(
        "mode",
        nargs="?",
        choices=["test"],
        help="Set to 'test' to bypass vision and type raw messages manually.",
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
        "--max-speed",
        type=int,
        default=255,
        help="Maximum PWM value sent to each motor [0..255].",
    )
    parser.add_argument(
        "--command-rate-hz",
        type=float,
        default=20.0,
        help="Minimum motor-speed send rate for keepalive.",
    )
    parser.add_argument(
        "--green-min-area",
        type=float,
        default=500.0,
        help="Minimum green contour area required to send D.",
    )
    parser.add_argument(
        "--blue-min-area",
        type=float,
        default=500.0,
        help="Minimum blue contour area required to send B.",
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


def clamp_speed(value):
    return max(0, min(255, int(round(value))))


def motor_speeds_from_centroid(cx, frame_w, left_bound, right_bound, max_speed):
    full = clamp_speed(max_speed)
    if cx is None:
        return 0, 0, "STOP"

    if cx <= left_bound:
        if left_bound <= 0:
            return 0, full, "TURN LEFT"
        left_ratio = np.clip(float(cx) / float(left_bound), 0.0, 1.0)
        return clamp_speed(full * left_ratio), full, "TURN LEFT"

    if cx >= right_bound:
        right_edge = max(0.0, float(frame_w - 1))
        right_span = max(1.0, right_edge - float(right_bound))
        right_ratio = np.clip((right_edge - float(cx)) / right_span, 0.0, 1.0)
        return full, clamp_speed(full * right_ratio), "TURN RIGHT"

    return full, full, "FORWARD"


def write_message(bus, addr, message):
    payload = message.encode("ascii")
    if not payload:
        raise ValueError("Cannot send an empty I2C message.")
    bus.i2c_rdwr(i2c_msg.write(addr, payload))


def send_motor_speeds(bus, addr, left_speed, right_speed):
    write_message(bus, addr, "left {}".format(clamp_speed(left_speed)))
    write_message(bus, addr, "right {}".format(clamp_speed(right_speed)))


def send_stop(bus, addr):
    send_motor_speeds(bus, addr, 0, 0)


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


def detect_color(image, lower_hsv, upper_hsv, min_area):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    color_mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    color_mask = cv2.erode(color_mask, GREEN_MORPH_KERNEL, iterations=1)
    color_mask = cv2.dilate(color_mask, GREEN_MORPH_KERNEL, iterations=2)
    contours, _ = unpack_contours(
        cv2.findContours(color_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    )
    if not contours:
        return False, 0.0

    largest_area = max(float(cv2.contourArea(contour)) for contour in contours)
    return largest_area >= min_area, largest_area


def detect_green(image, min_area):
    return detect_color(image, GREEN_LOWER_HSV, GREEN_UPPER_HSV, min_area)


def detect_blue(image, min_area):
    return detect_color(image, BLUE_LOWER_HSV, BLUE_UPPER_HSV, min_area)


def run_test_mode(args):
    bus = None
    try:
        bus = SMBus(args.i2c_bus)
        time.sleep(0.1)
        send_stop(bus, args.i2c_address)
        print("Test mode: type messages like 'left 255', 'right 128', 'D', or 'B'. Type Q to quit.")

        while True:
            try:
                raw = input("cmd> ")
            except (EOFError, KeyboardInterrupt):
                print()
                break

            message = raw.strip()
            if not message:
                continue
            if message.upper() in ("Q", "QUIT", "EXIT"):
                break

            write_message(bus, args.i2c_address, message)
            print("Sent {}".format(message))
    finally:
        try:
            if bus is not None:
                send_stop(bus, args.i2c_address)
        except Exception:
            pass
        if bus is not None:
            bus.close()


def main():
    args = parse_args()

    if not (0x03 <= args.i2c_address <= 0x77):
        raise ValueError("--i2c-address must be in the valid 7-bit range 0x03-0x77.")
    if not (0 <= args.max_speed <= 255):
        raise ValueError("--max-speed must be in [0, 255].")

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
    last_sent_left = None
    last_sent_right = None
    last_send_time = 0.0
    near_cx_ema = None
    look_cx_ema = None
    miss_streak = 0
    green_detected_prev = False
    blue_detected_prev = False

    try:
        bus = SMBus(args.i2c_bus)
        time.sleep(0.1)
        send_stop(bus, args.i2c_address)

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

            left_speed, right_speed, state_label = motor_speeds_from_centroid(
                fused_cx, frame_w, left_bound, right_bound, args.max_speed
            )
            green_detected, green_area = detect_green(image, args.green_min_area)
            blue_detected, blue_area = detect_blue(image, args.blue_min_area)

            now = time.monotonic()
            if (
                left_speed != last_sent_left
                or right_speed != last_sent_right
                or (now - last_send_time) >= min_interval
            ):
                try:
                    send_motor_speeds(bus, args.i2c_address, left_speed, right_speed)
                except OSError as exc:
                    raise RuntimeError(
                        "I2C write failed on bus {} to address {}.".format(
                            args.i2c_bus, hex(args.i2c_address)
                        )
                    ) from exc
                last_sent_left = left_speed
                last_sent_right = right_speed
                last_send_time = now

            if green_detected and not green_detected_prev:
                try:
                    write_message(bus, args.i2c_address, "D")
                except OSError as exc:
                    raise RuntimeError(
                        "I2C write failed on bus {} to address {}.".format(
                            args.i2c_bus, hex(args.i2c_address)
                        )
                    ) from exc
            green_detected_prev = green_detected

            if blue_detected and not blue_detected_prev:
                try:
                    write_message(bus, args.i2c_address, "B")
                except OSError as exc:
                    raise RuntimeError(
                        "I2C write failed on bus {} to address {}.".format(
                            args.i2c_bus, hex(args.i2c_address)
                        )
                    ) from exc
            blue_detected_prev = blue_detected

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
                    "STATE: {}".format(state_label),
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
                    "near_x={} area={:.0f}  look_x={} area={:.0f}  green={:.0f}  blue={:.0f}".format(
                        "-" if near_cx is None else near_cx,
                        near_area,
                        "-" if look_cx is None else look_cx,
                        look_area,
                        green_area,
                        blue_area,
                    ),
                    (8, 80),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (220, 220, 220),
                    1,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    image,
                    "mode={}  D={}  B={}".format(
                        args.threshold_mode,
                        "YES" if green_detected else "NO",
                        "YES" if blue_detected else "NO",
                    ),
                    (8, 104),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 220, 120) if (green_detected or blue_detected) else (220, 220, 220),
                    1,
                    cv2.LINE_AA,
                )
                cv2.imshow("img", image)

                # If user closes the window (X button), stop robot and exit.
                if cv2.getWindowProperty("img", cv2.WND_PROP_VISIBLE) < 1:
                    send_stop(bus, args.i2c_address)
                    break

                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    send_stop(bus, args.i2c_address)
                    break
    finally:
        try:
            if bus is not None:
                send_stop(bus, args.i2c_address)
        except Exception:
            pass
        if bus is not None:
            bus.close()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
