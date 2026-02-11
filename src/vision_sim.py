import argparse
import time

import cv2

from vision import compute_features


def parse_args():
    p = argparse.ArgumentParser(description="Live vision simulation/debug for the Pi camera pipeline.")
    p.add_argument("--camera", type=int, default=0, help="VideoCapture index (Logitech often 0 or 1).")
    p.add_argument("--fps", type=float, default=30.0, help="Target processing FPS.")
    p.add_argument("--width", type=int, default=0, help="Optional capture width.")
    p.add_argument("--height", type=int, default=0, help="Optional capture height.")
    p.add_argument("--roi", type=float, default=0.6, help="Bottom fraction of frame to use as ROI (0-1).")
    p.add_argument("--thresh", type=int, default=70, help="Binary inverse threshold value.")
    p.add_argument("--row-frac", type=float, default=0.02, help="Row occupancy fraction threshold.")
    p.add_argument("--show-bw", action="store_true", help="Show thresholded ROI window.")
    return p.parse_args()


def main():
    args = parse_args()
    period = 1.0 / max(1.0, args.fps)

    cap = cv2.VideoCapture(args.camera)
    if args.width:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    if args.height:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)

    last = 0.0
    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                continue

            lk, valid, len_pct = compute_features(
                frame,
                roi_bottom_ratio=args.roi,
                thresh_val=args.thresh,
                row_occupancy_frac=args.row_frac,
            )

            h, w = frame.shape[:2]
            y0 = int(h * (1 - args.roi))
            cv2.line(frame, (0, y0), (w - 1, y0), (0, 255, 255), 2)

            color = (0, 255, 0) if valid else (0, 0, 255)
            cv2.putText(
                frame,
                f"lk={lk:.2f} valid={valid} len%={len_pct:.1f}",
                (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                color,
                2,
            )

            cv2.imshow("vision_sim", frame)

            if args.show_bw:
                roi = frame[y0:h, :]
                gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                _, bw = cv2.threshold(gray, args.thresh, 255, cv2.THRESH_BINARY_INV)
                bw = cv2.medianBlur(bw, 5)
                cv2.imshow("vision_bw", bw)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

            now = time.time()
            sleep_s = period - (now - last)
            if sleep_s > 0:
                time.sleep(sleep_s)
            last = now
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
