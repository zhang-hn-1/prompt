"""Only per-hypothesis availability changes; no RGB, cost or decoder changes."""
import numpy as np
import cv2

def self_support(support, query_CAD, partner_CAD, disparities):
    s = np.asarray(support); a = np.asarray(query_CAD); b = np.asarray(partner_CAD); ds = np.asarray(disparities)
    if s.ndim != 3 or s.dtype != bool or a.dtype != bool or b.dtype != bool or a.shape != b.shape or a.shape != s.shape[:2]:
        raise ValueError('HWD bool support and two HW bool CAD masks required')
    if ds.ndim != 1 or len(ds) != s.shape[-1] or not np.issubdtype(ds.dtype, np.integer):
        raise ValueError('integer disparity axis required')
    # Coordinates are np.rot90 RGB: BT/block uses x+/-4, y+/-3.
    kernel = np.ones((7, 9), np.uint8)
    own = ~cv2.dilate(a.astype(np.uint8), kernel).astype(bool)
    other = ~cv2.dilate(b.astype(np.uint8), kernel).astype(bool)
    H, W = a.shape; Y, X = np.indices((H, W)); result = s.copy()
    for k, d in enumerate(ds.tolist()):
        xp = X - d
        available = (xp >= 0) & (xp < W)
        result[:, :, k] &= own & available & other[Y, np.clip(xp, 0, W - 1)]
    return result
