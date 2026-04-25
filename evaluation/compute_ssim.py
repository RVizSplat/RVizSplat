"""SSIM computation on numpy image arrays (uint8, HxWxC or HxW). Pure numpy.
Higher is better"""

import numpy as np

_WIN_SIZE = 11
_SIGMA = 1.5
_C1 = (0.01 * 255) ** 2
_C2 = (0.03 * 255) ** 2


def _make_gaussian_kernel(size: int, sigma: float) -> np.ndarray:
    x = np.arange(size) - size // 2
    k = np.exp(-(x ** 2) / (2.0 * sigma ** 2))
    return k / k.sum()


def _gaussian_filter2d(img: np.ndarray, kernel1d: np.ndarray) -> np.ndarray:
    pad = len(kernel1d) // 2
    row_filtered = np.apply_along_axis(
        lambda row: np.convolve(np.pad(row, pad, mode="reflect"), kernel1d, mode="valid"),
        axis=1, arr=img.astype(np.float64),
    )
    return np.apply_along_axis(
        lambda col: np.convolve(np.pad(col, pad, mode="reflect"), kernel1d, mode="valid"),
        axis=0, arr=row_filtered,
    )


def _ssim_channel(ref: np.ndarray, ev: np.ndarray, kernel1d: np.ndarray) -> float:
    ref_f, ev_f = ref.astype(np.float64), ev.astype(np.float64)

    mu1 = _gaussian_filter2d(ref_f, kernel1d)
    mu2 = _gaussian_filter2d(ev_f, kernel1d)

    mu1_sq, mu2_sq, mu1_mu2 = mu1 * mu1, mu2 * mu2, mu1 * mu2

    sigma1_sq = _gaussian_filter2d(ref_f * ref_f, kernel1d) - mu1_sq
    sigma2_sq = _gaussian_filter2d(ev_f * ev_f, kernel1d) - mu2_sq
    sigma12   = _gaussian_filter2d(ref_f * ev_f, kernel1d) - mu1_mu2

    numerator   = (2.0 * mu1_mu2 + _C1) * (2.0 * sigma12   + _C2)
    denominator = (mu1_sq + mu2_sq + _C1) * (sigma1_sq + sigma2_sq + _C2)

    return float(np.mean(numerator / denominator))


def compute_ssim(ref: np.ndarray, ev: np.ndarray) -> float:
    kernel1d = _make_gaussian_kernel(_WIN_SIZE, _SIGMA)
    if ref.ndim == 2:
        return _ssim_channel(ref, ev, kernel1d)
    return float(np.mean([_ssim_channel(ref[..., c], ev[..., c], kernel1d) for c in range(ref.shape[2])]))
