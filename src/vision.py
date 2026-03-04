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
    Return a denoised binary mask for a line of any color by selecting the
    better of dark-line and bright-line threshold candidates.

    Behavior:
    - thresh_val <= 0: use Otsu threshold for both polarities.
    - thresh_val > 0: use manual threshold for both polarities.
    - pick lower-density valid mask to avoid over-segmentation.
    """
    gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    if thresh_val <= 0:
        _, dark = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        _, bright = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    else:
        t = int(np.clip(thresh_val, 1, 254))
        _, dark = cv2.threshold(gray, t, 255, cv2.THRESH_BINARY_INV)
        _, bright = cv2.threshold(gray, t, 255, cv2.THRESH_BINARY)

    dark = _cleanup_mask(dark)
    bright = _cleanup_mask(bright)

    # Prefer the candidate with less over-segmentation.
    dark_ratio = float((dark > 0).mean())
    bright_ratio = float((bright > 0).mean())

    dark_ok = dark_ratio < 0.60
    bright_ok = bright_ratio < 0.60
    if dark_ok and bright_ok:
        return dark if dark_ratio <= bright_ratio else bright
    if dark_ok:
        return dark
    if bright_ok:
        return bright

    # Fallback: pick the less-dense candidate even if both are noisy.
    return dark if dark_ratio <= bright_ratio else bright


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


def compute_l2(
    frame: np.ndarray,
    roi_bottom_ratio: float = 0.6,
    thresh_val: int = 0,
    row_occupancy_frac: float = 0.02,
) -> Tuple[float, int]:
    """Convenience wrapper for callers that only need (L2_percent, valid)."""
    _, valid, l2_pct = compute_features(
        frame,
        roi_bottom_ratio=roi_bottom_ratio,
        thresh_val=thresh_val,
        row_occupancy_frac=row_occupancy_frac,
    )
    return l2_pct, valid
