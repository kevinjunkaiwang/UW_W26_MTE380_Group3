import cv2
import numpy as np
from typing import Tuple


def compute_features(
    frame: np.ndarray,
    roi_bottom_ratio: float = 0.6,
    thresh_val: int = 70,
    row_occupancy_frac: float = 0.02,
) -> Tuple[float, int, float]:
    """
    Compute look-ahead metric and line length percentage.

    Returns:
        lk_norm (0..1): how far up the ROI the line extends (higher = further ahead).
        valid (0/1): line detected flag.
        len_pct (0..100): fraction of ROI rows containing line pixels.
    """
    h, w = frame.shape[:2]
    y0 = int(h * (1 - roi_bottom_ratio))
    roi = frame[y0:h, :]

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    _, bw = cv2.threshold(gray, thresh_val, 255, cv2.THRESH_BINARY_INV)
    bw = cv2.medianBlur(bw, 5)

    row_sum = (bw > 0).sum(axis=1)
    thresh = int(row_occupancy_frac * w)
    rows = np.where(row_sum > thresh)[0]

    if len(rows) == 0:
        return 0.0, 0, 0.0

    top = int(rows.min())
    roi_h = bw.shape[0]
    lk_norm = 1.0 - (top / max(1, roi_h - 1))
    lk_norm = float(np.clip(lk_norm, 0.0, 1.0))

    len_pct = float(len(rows) / roi_h * 100.0)
    len_pct = float(np.clip(len_pct, 0.0, 100.0))
    return lk_norm, 1, len_pct
