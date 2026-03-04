import argparse
import time

import cv2

from vision import compute_features


def parse_args():
    p = argparse.ArgumentParser(description="Headless webcam L2 monitor for Raspberry Pi.")
    p.add_argument("--camera", type=int, default=0, help="VideoCapture index.")
    p.add_argument("--width", type=int, default=0, help="Optional capture width.")
    p.add_argument("--height", type=int, default=0, help="Optional capture height.")
    p.add_argument("--roi", type=float, default=0.6, help="Bottom fraction of frame used for L2.")
    p.add_argument("--thresh", type=int, default=0, help="Line threshold (0 = auto Otsu).")
    p.add_argument("--row-frac", type=float, default=0.02, help="Row occupancy fraction threshold.")
    p.add_argument("--print-hz", type=float, default=10.0, help="How often to print L2 to terminal.")
    return p.parse_args()


def _open_camera(index: int) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        cap = cv2.VideoCapture(index)
    return cap


def main():
    args = parse_args()
    cap = _open_camera(args.camera)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera index {args.camera}.")

    if args.width > 0:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    if args.height > 0:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    period = 1.0 / max(1.0, float(args.print_hz))
    next_print = time.monotonic()

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.005)
                continue

            _, valid, l2_pct = compute_features(
                frame,
                roi_bottom_ratio=args.roi,
                thresh_val=args.thresh,
                row_occupancy_frac=args.row_frac,
            )
            now = time.monotonic()
            if now >= next_print:
                print(f"valid={int(valid)} l2_pct={l2_pct:5.1f}%")
                next_print = now + period
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()


if __name__ == "__main__":
    main()
