import argparse
import time
import cv2

from serial_link import SerialSender
from vision import compute_features


def parse_args():
    p = argparse.ArgumentParser(description="Pi vision sender for line following.")
    p.add_argument("--camera", type=int, default=0, help="VideoCapture index.")
    p.add_argument("--port", type=str, default="/dev/ttyACM0", help="Serial port to Arduino.")
    p.add_argument("--baud", type=int, default=115200, help="Serial baud rate.")
    p.add_argument("--fps", type=float, default=30.0, help="Send rate in Hz.")
    p.add_argument("--display", action="store_true", help="Show debug window.")
    p.add_argument("--roi", type=float, default=0.6, help="Bottom fraction of frame to use as ROI (0-1).")
    return p.parse_args()


def main():
    args = parse_args()
    send_period = 1.0 / max(1.0, args.fps)

    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FPS, args.fps)

    ser = SerialSender(args.port, args.baud)
    ser.open()

    last_send = 0.0
    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                continue

            lk, valid, len_pct = compute_features(frame, roi_bottom_ratio=args.roi)

            now = time.time()
            if now - last_send >= send_period:
                msg = f"L,{lk:.3f},{valid}\n"
                ser.send_line(msg)
                last_send = now

            if args.display:
                cv2.putText(frame, f"lk={lk:.2f} valid={valid} len%={len_pct:.1f}", (20, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.imshow("vision", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    finally:
        ser.close()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
