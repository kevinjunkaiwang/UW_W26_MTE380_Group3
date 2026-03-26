"""
Laptop-side vision simulator for the line-following pipeline.

This uses the same red/green/blue detection logic and steering geometry as the
Pi script, but only shows the camera preview and inferred command state.
"""

import argparse
import time

import cv2
import numpy as np


def parse_args():
    parser = argparse.ArgumentParser(
        description="Live camera simulator for the line-following vision pipeline."
    )
    parser.add_argument("--camera-index", type=int, default=0, help="VideoCapture index.")
    parser.add_argument("--width", type=int, default=1280, help="Requested capture width.")
    parser.add_argument("--height", type=int, default=720, help="Requested capture height.")
    parser.add_argument("--cam-fps", type=float, default=30.0, help="Requested camera FPS.")
    parser.add_argument(
        "--red-h1-low",
        type=int,
        default=0,
        help="Lower hue bound for the first red HSV range [0..179].",
    )
    parser.add_argument(
        "--red-h1-high",
        type=int,
        default=12,
        help="Upper hue bound for the first red HSV range [0..179].",
    )
    parser.add_argument(
        "--red-h2-low",
        type=int,
        default=170,
        help="Lower hue bound for the second red HSV range [0..179].",
    )
    parser.add_argument(
        "--red-h2-high",
        type=int,
        default=179,
        help="Upper hue bound for the second red HSV range [0..179].",
    )
    parser.add_argument(
        "--red-s-min",
        type=int,
        default=90,
        help="Minimum saturation required for red pixels [0..255].",
    )
    parser.add_argument(
        "--red-v-min",
        type=int,
        default=50,
        help="Minimum value/brightness required for red pixels [0..255].",
    )
    parser.add_argument(
        "--green-h-low",
        type=int,
        default=75,
        help="Lower hue bound for green HSV range [0..179].",
    )
    parser.add_argument(
        "--green-h-high",
        type=int,
        default=100,
        help="Upper hue bound for green HSV range [0..179].",
    )
    parser.add_argument(
        "--green-s-min",
        type=int,
        default=40,
        help="Minimum saturation required for green pixels [0..255].",
    )
    parser.add_argument(
        "--green-v-min",
        type=int,
        default=50,
        help="Minimum value/brightness required for green pixels [0..255].",
    )
    parser.add_argument(
        "--blue-h-low",
        type=int,
        default=105,
        help="Lower hue bound for blue HSV range [0..179].",
    )
    parser.add_argument(
        "--blue-h-high",
        type=int,
        default=140,
        help="Upper hue bound for blue HSV range [0..179].",
    )
    parser.add_argument(
        "--blue-s-min",
        type=int,
        default=60,
        help="Minimum saturation required for blue pixels [0..255].",
    )
    parser.add_argument(
        "--blue-v-min",
        type=int,
        default=50,
        help="Minimum value/brightness required for blue pixels [0..255].",
    )
    parser.add_argument(
        "--min-area",
        type=float,
        default=120.0,
        help="Minimum contour area accepted inside a scan band.",
    )
    parser.add_argument(
        "--green-min-area",
        type=float,
        default=20000.0,
        help="Minimum contour area required to trigger the green D event.",
    )
    parser.add_argument(
        "--blue-min-area",
        type=float,
        default=20000.0,
        help="Minimum contour area required to trigger the blue U event.",
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
        "--max-speed",
        type=int,
        default=255,
        help="Maximum drive speed attached to F/L/R commands [0..255].",
    )
    parser.add_argument(
        "--min-turn-speed",
        type=int,
        default=255,
        help="Minimum turn speed attached to L/R commands [0..255].",
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


def command_from_centroid(cx, frame_w, left_bound, right_bound, args):
    if cx is None:
        return "S"
    if cx <= left_bound:
        turn_span = max(1, left_bound)
        turn_ratio = min(1.0, (left_bound - cx) / float(turn_span))
        speed = args.min_turn_speed + turn_ratio * (args.max_speed - args.min_turn_speed)
        return "L {}".format(clamp_speed(speed))
    if cx >= right_bound:
        turn_span = max(1, frame_w - right_bound)
        turn_ratio = min(1.0, (cx - right_bound) / float(turn_span))
        speed = args.min_turn_speed + turn_ratio * (args.max_speed - args.min_turn_speed)
        return "R {}".format(clamp_speed(speed))
    return "F {}".format(clamp_speed(args.max_speed))


def motor_state_from_command(cmd):
    if cmd == "D":
        return 0, 0, "GREEN DETECTED"
    if cmd == "U":
        return 0, 0, "BLUE DETECTED"

    parts = cmd.split()
    if not parts:
        return 0, 0, "STOP"

    action = parts[0].upper()
    try:
        speed = clamp_speed(int(parts[1])) if len(parts) > 1 else 0
    except ValueError:
        speed = 0

    if action == "F":
        return speed, speed, "FORWARD"
    if action == "L":
        return 0, speed, "TURN LEFT"
    if action == "R":
        return speed, 0, "TURN RIGHT"
    return 0, 0, "STOP"


def image_hsv(image_bgr):
    blurred = cv2.GaussianBlur(image_bgr, (5, 5), 0)
    return cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)


def dual_hue_mask(hsv, h1_low, h1_high, h2_low, h2_high, s_min, v_min):
    lower_1 = np.array([h1_low, s_min, v_min], dtype=np.uint8)
    upper_1 = np.array([h1_high, 255, 255], dtype=np.uint8)
    lower_2 = np.array([h2_low, s_min, v_min], dtype=np.uint8)
    upper_2 = np.array([h2_high, 255, 255], dtype=np.uint8)

    mask_1 = cv2.inRange(hsv, lower_1, upper_1)
    mask_2 = cv2.inRange(hsv, lower_2, upper_2)
    return cv2.bitwise_or(mask_1, mask_2)


def single_hue_mask(hsv, h_low, h_high, s_min, v_min):
    lower = np.array([h_low, s_min, v_min], dtype=np.uint8)
    upper = np.array([h_high, 255, 255], dtype=np.uint8)
    return cv2.inRange(hsv, lower, upper)


def red_mask(hsv, args):
    return dual_hue_mask(
        hsv,
        args.red_h1_low,
        args.red_h1_high,
        args.red_h2_low,
        args.red_h2_high,
        args.red_s_min,
        args.red_v_min,
    )


def green_mask(hsv, args):
    return single_hue_mask(
        hsv, args.green_h_low, args.green_h_high, args.green_s_min, args.green_v_min
    )


def blue_mask(hsv, args):
    return single_hue_mask(
        hsv, args.blue_h_low, args.blue_h_high, args.blue_s_min, args.blue_v_min
    )


def largest_contour_info(mask):
    contours, _ = unpack_contours(
        cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    )
    if not contours:
        return 0.0, None, None

    contour = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(contour))
    bbox = cv2.boundingRect(contour) if area > 0.0 else None
    return area, contour, bbox


def event_message_from_areas(green_area, blue_area, green_min_area, blue_min_area):
    if green_area >= green_min_area and green_area >= blue_area:
        return "D"
    if blue_area >= blue_min_area:
        return "U"
    return None


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


def analyze_frame(image, args, kernel, near_cx_ema, look_cx_ema, miss_streak):
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

    hsv = image_hsv(image)
    mask = red_mask(hsv, args)
    mask = cv2.erode(mask, kernel, iterations=2)
    mask = cv2.dilate(mask, kernel, iterations=2)
    green = green_mask(hsv, args)
    green = cv2.erode(green, kernel, iterations=1)
    green = cv2.dilate(green, kernel, iterations=1)
    blue = blue_mask(hsv, args)
    blue = cv2.erode(blue, kernel, iterations=1)
    blue = cv2.dilate(blue, kernel, iterations=1)

    green_area, _, green_box = largest_contour_info(green)
    blue_area, _, blue_box = largest_contour_info(blue)

    near_cx, near_cy, near_area = find_band_center(mask, near_y0, near_y1, args.min_area)
    look_cx, look_cy, look_area = find_band_center(mask, look_y0, look_y1, args.min_area)

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

    green_detected = green_area >= args.green_min_area
    blue_detected = blue_area >= args.blue_min_area
    event_cmd = event_message_from_areas(
        green_area, blue_area, args.green_min_area, args.blue_min_area
    )
    if event_cmd is not None:
        cmd = event_cmd
    else:
        cmd = command_from_centroid(fused_cx, frame_w, left_bound, right_bound, args)

    left_speed, right_speed, state_label = motor_state_from_command(cmd)
    left_pct = int(round((left_speed / 255.0) * 100))
    right_pct = int(round((right_speed / 255.0) * 100))

    return {
        "frame_h": frame_h,
        "frame_w": frame_w,
        "left_bound": left_bound,
        "right_bound": right_bound,
        "search_y0": search_y0,
        "near_y0": near_y0,
        "near_y1": near_y1,
        "look_y0": look_y0,
        "look_y1": look_y1,
        "near_cx": near_cx,
        "near_cy": near_cy,
        "near_area": near_area,
        "look_cx": look_cx,
        "look_cy": look_cy,
        "look_area": look_area,
        "fused_cx": fused_cx,
        "green_area": green_area,
        "blue_area": blue_area,
        "green_detected": green_detected,
        "blue_detected": blue_detected,
        "green_box": green_box,
        "blue_box": blue_box,
        "event_cmd": event_cmd,
        "cmd": cmd,
        "left_speed": left_speed,
        "right_speed": right_speed,
        "left_pct": left_pct,
        "right_pct": right_pct,
        "state_label": state_label,
    }, near_cx_ema, look_cx_ema, miss_streak


def command_color(cmd):
    action = cmd.split()[0].upper() if cmd else "S"
    if action == "F":
        return (0, 255, 0)
    if action in ("L", "R"):
        return (0, 255, 255)
    if action == "D":
        return (0, 220, 0)
    if action == "U":
        return (255, 140, 0)
    return (0, 80, 255)


def draw_detection_box(image, bbox, label, color):
    if bbox is None:
        return

    x, y, w, h = bbox
    cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
    text_y = max(18, y - 8)
    cv2.putText(
        image,
        label,
        (x, text_y),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        color,
        2,
        cv2.LINE_AA,
    )


def draw_status_panel(image, lines):
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.55
    thickness = 2
    padding = 10
    line_gap = 8
    margin = 10
    text_sizes = [cv2.getTextSize(text, font, font_scale, thickness)[0] for text, _ in lines]
    panel_w = max((size[0] for size in text_sizes), default=0) + (padding * 2)
    panel_h = (
        sum(size[1] for size in text_sizes)
        + max(0, len(text_sizes) - 1) * line_gap
        + (padding * 2)
    )
    x1 = image.shape[1] - margin
    x0 = max(0, x1 - panel_w)
    y0 = margin
    y1 = y0 + panel_h
    cv2.rectangle(image, (x0, y0), (x1, y1), (25, 25, 25), -1)
    cv2.rectangle(image, (x0, y0), (x1, y1), (160, 160, 160), 1)

    y = y0 + padding
    for (text, color), size in zip(lines, text_sizes):
        y += size[1]
        cv2.putText(
            image,
            text,
            (x0 + padding, y),
            font,
            font_scale,
            color,
            thickness,
            cv2.LINE_AA,
        )
        y += line_gap


def annotate_frame(image, analysis, args):
    frame_h = analysis["frame_h"]
    frame_w = analysis["frame_w"]
    left_bound = analysis["left_bound"]
    right_bound = analysis["right_bound"]
    search_y0 = analysis["search_y0"]
    near_y0 = analysis["near_y0"]
    near_y1 = analysis["near_y1"]
    look_y0 = analysis["look_y0"]
    look_y1 = analysis["look_y1"]
    near_cx = analysis["near_cx"]
    near_cy = analysis["near_cy"]
    look_cx = analysis["look_cx"]
    look_cy = analysis["look_cy"]
    fused_cx = analysis["fused_cx"]

    cv2.line(image, (left_bound, 0), (left_bound, frame_h - 1), (255, 0, 0), 1)
    cv2.line(image, (right_bound, 0), (right_bound, frame_h - 1), (255, 0, 0), 1)
    if search_y0 > 0:
        cv2.line(image, (0, search_y0), (frame_w - 1, search_y0), (80, 80, 80), 1)
    cv2.rectangle(image, (0, near_y0), (frame_w - 1, near_y1 - 1), (0, 180, 255), 1)
    cv2.rectangle(image, (0, look_y0), (frame_w - 1, look_y1 - 1), (255, 180, 0), 1)
    if near_cx is not None and near_cy is not None:
        cv2.circle(image, (near_cx, near_cy), 4, (0, 165, 255), -1)
    if look_cx is not None and look_cy is not None:
        cv2.circle(image, (look_cx, look_cy), 4, (255, 165, 0), -1)
    if fused_cx is not None:
        cv2.line(image, (fused_cx, 0), (fused_cx, frame_h - 1), (255, 255, 255), 1)

    if analysis["green_detected"]:
        draw_detection_box(
            image,
            analysis["green_box"],
            "GREEN {:.0f}".format(analysis["green_area"]),
            (0, 220, 0),
        )
    if analysis["blue_detected"]:
        draw_detection_box(
            image,
            analysis["blue_box"],
            "BLUE {:.0f}".format(analysis["blue_area"]),
            (255, 140, 0),
        )

    cv2.putText(
        image,
        "SIM: {} ({})".format(analysis["cmd"], analysis["state_label"]),
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
            analysis["left_speed"],
            analysis["left_pct"],
            analysis["right_speed"],
            analysis["right_pct"],
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
        "near_x={} area={:.0f}  look_x={} area={:.0f}  mask=red".format(
            "-" if near_cx is None else near_cx,
            analysis["near_area"],
            "-" if look_cx is None else look_cx,
            analysis["look_area"],
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
        "green_area={:.0f}/{:.0f}  blue_area={:.0f}/{:.0f}".format(
            analysis["green_area"],
            args.green_min_area,
            analysis["blue_area"],
            args.blue_min_area,
        ),
        (8, 108),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (180, 255, 180),
        1,
        cv2.LINE_AA,
    )

    panel_lines = [
        ("CMD: {}".format(analysis["cmd"]), command_color(analysis["cmd"])),
        (
            "GREEN: {}".format("DETECTED" if analysis["green_detected"] else "clear"),
            (0, 220, 0) if analysis["green_detected"] else (120, 150, 120),
        ),
        (
            "BLUE: {}".format("DETECTED" if analysis["blue_detected"] else "clear"),
            (255, 140, 0) if analysis["blue_detected"] else (120, 120, 120),
        ),
        (
            "EVENT: {}".format(analysis["event_cmd"] if analysis["event_cmd"] else "-"),
            (220, 220, 220),
        ),
    ]
    draw_status_panel(image, panel_lines)


def validate_args(args):
    for name in (
        "red_h1_low",
        "red_h1_high",
        "red_h2_low",
        "red_h2_high",
        "green_h_low",
        "green_h_high",
        "blue_h_low",
        "blue_h_high",
    ):
        value = getattr(args, name)
        if not (0 <= value <= 179):
            raise ValueError("--{} must be in [0, 179].".format(name.replace("_", "-")))
    for name in (
        "red_s_min",
        "red_v_min",
        "green_s_min",
        "green_v_min",
        "blue_s_min",
        "blue_v_min",
    ):
        value = getattr(args, name)
        if not (0 <= value <= 255):
            raise ValueError("--{} must be in [0, 255].".format(name.replace("_", "-")))
    if args.red_h1_low > args.red_h1_high:
        raise ValueError("--red-h1-low must be <= --red-h1-high.")
    if args.red_h2_low > args.red_h2_high:
        raise ValueError("--red-h2-low must be <= --red-h2-high.")
    if args.green_h_low > args.green_h_high:
        raise ValueError("--green-h-low must be <= --green-h-high.")
    if args.blue_h_low > args.blue_h_high:
        raise ValueError("--blue-h-low must be <= --blue-h-high.")
    if not (0.0 < args.roi_bottom_ratio <= 1.0):
        raise ValueError("--roi-bottom-ratio must be in (0, 1].")
    if args.min_area < 0.0:
        raise ValueError("--min-area must be >= 0.")
    if args.green_min_area < 0.0:
        raise ValueError("--green-min-area must be >= 0.")
    if args.blue_min_area < 0.0:
        raise ValueError("--blue-min-area must be >= 0.")
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
    if not (0 <= args.max_speed <= 255):
        raise ValueError("--max-speed must be in [0, 255].")
    if not (0 <= args.min_turn_speed <= args.max_speed):
        raise ValueError("--min-turn-speed must be in [0, --max-speed].")


def main():
    args = parse_args()
    validate_args(args)

    cap = open_camera(args.camera_index, args.width, args.height, args.cam_fps)
    if not cap.isOpened():
        raise RuntimeError(
            "Could not open camera index {}. Try another --camera-index.".format(
                args.camera_index
            )
        )

    cv2.namedWindow("line-follow-sim", cv2.WINDOW_NORMAL)
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if actual_width <= 0:
        actual_width = args.width if args.width > 0 else 960
    if actual_height <= 0:
        actual_height = args.height if args.height > 0 else 540
    cv2.resizeWindow("line-follow-sim", actual_width, actual_height)

    kernel = np.ones((3, 3), dtype=np.uint8)
    near_cx_ema = None
    look_cx_ema = None
    miss_streak = 0
    camera_read_failures = 0

    try:
        while True:
            ok, image = cap.read()
            if not ok or image is None:
                camera_read_failures += 1
                near_cx_ema = None
                look_cx_ema = None
                miss_streak = 0
                if camera_read_failures == 1:
                    print("Camera read failed in simulator.", flush=True)
                time.sleep(0.05)
                if camera_read_failures >= 10:
                    raise RuntimeError(
                        "Camera read failed 10 consecutive times in simulator; exiting."
                    )
                continue

            if camera_read_failures > 0:
                print(
                    "Camera read recovered after {} consecutive failure(s).".format(
                        camera_read_failures
                    ),
                    flush=True,
                )
                camera_read_failures = 0

            analysis, near_cx_ema, look_cx_ema, miss_streak = analyze_frame(
                image, args, kernel, near_cx_ema, look_cx_ema, miss_streak
            )
            annotate_frame(image, analysis, args)
            cv2.imshow("line-follow-sim", image)

            # If the user closes the window, stop cleanly.
            if cv2.getWindowProperty("line-follow-sim", cv2.WND_PROP_VISIBLE) < 1:
                break
            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
