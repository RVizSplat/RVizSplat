"""PSNR computation on numpy image arrays (uint8, HxWxC).
Higher is better"""

import numpy as np


def compute_psnr(ref: np.ndarray, ev: np.ndarray) -> float:
    mse = np.mean((ref.astype(np.float64) - ev.astype(np.float64)) ** 2)
    if mse == 0:
        return float("inf")
    return 10.0 * np.log10(255.0**2 / mse)
