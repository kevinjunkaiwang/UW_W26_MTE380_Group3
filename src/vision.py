"""
Vision feature extraction used by the Pi controller.

This module converts a camera frame into fuzzy-ready visibility metrics:
- valid: whether a line is detected in the ROI
- l2_pct: line length percentage in ROI (used as fuzzy X2)
- lk_norm: auxiliary look-ahead metric for debug/inspection
"""

import cv2
import numpy as np
from typing import Tuple


def _cleanup_mask(mask: np.ndarray) -> np.ndarray:
    """Apply light morphology to suppress speckle and close tiny gaps."""
    kernel = np.ones((3, 3), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
    return mask


def _line_mask_any_color(roi_bgr: np.ndarray, thresh_val: int) -> np.ndarray:
    """
    Return a denoised binary mask for a green/teal line.

    Behavior:
    - If `thresh_val <= 0`, use default saturation/value floors.
    - If `thresh_val > 0`, use it to tighten saturation/value requirements.
    """
    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    hsv = cv2.GaussianBlur(hsv, (5, 5), 0)

    if thresh_val <= 0:
        min_sat = 45
        min_val = 35
    else:
        min_sat = int(np.clip(thresh_val, 35, 255))
        min_val = int(np.clip(thresh_val + 5, 25, 255))

    # Green plus teal/cyan bands to catch camera-dependent hue shifts.
    lower_green = np.array([30, min_sat, min_val], dtype=np.uint8)
    upper_green = np.array([95, 255, 255], dtype=np.uint8)
    lower_teal = np.array([90, min_sat, min_val], dtype=np.uint8)
    upper_teal = np.array([120, 255, 255], dtype=np.uint8)

    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_teal = cv2.inRange(hsv, lower_teal, upper_teal)
    mask = cv2.bitwise_or(mask_green, mask_teal)
    return _cleanup_mask(mask)


def compute_features(
    frame: np.ndarray,
    roi_bottom_ratio: float = 0.6,
    thresh_val: int = 0,
    row_occupancy_frac: float = 0.02,
) -> Tuple[float, int, float]:
    """
    Compute line look-ahead metric and L2 line-length percentage from one frame.

    Returns:
        lk_norm (0..1): how far up the ROI the line extends.
        valid (0/1): line detected flag.
        l2_pct (0..100): fraction of ROI rows containing line pixels.
            This corresponds to fuzzy input X2/L2 (line length percentage).
    """
    h, w = frame.shape[:2]
    # Restrict processing to bottom ROI where near-field tracking information lives.
    ratio = float(np.clip(roi_bottom_ratio, 0.05, 1.0))
    y0 = int(h * (1 - ratio))
    roi = frame[y0:h, :]

    mask = _line_mask_any_color(roi, thresh_val=thresh_val)

    row_sum = (mask > 0).sum(axis=1)
    # A row is considered part of line length if enough pixels are active.
    thresh = max(1, int(row_occupancy_frac * w))
    rows = np.where(row_sum > thresh)[0]

    if len(rows) == 0:
        return 0.0, 0, 0.0

    top = int(rows.min())
    roi_h = max(1, mask.shape[0])
    lk_norm = 1.0 - (top / max(1, roi_h - 1))
    lk_norm = float(np.clip(lk_norm, 0.0, 1.0))

    l2_pct = float(len(rows) / roi_h * 100.0)
    l2_pct = float(np.clip(l2_pct, 0.0, 100.0))
    return lk_norm, 1, l2_pct

