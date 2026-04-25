"""LPIPS computation using torchvision pretrained features + official v0.1 linear weights.

On first use per backbone, downloads the ~5 KB weight file from the official
LPIPS repo (github.com/richzhang/PerceptualSimilarity) and caches it under
~/.cache/lpips/v0.1/.  Subsequent calls use the cached file.

The per-layer linear weights are 1x1 convs applied to the squared normalised
feature differences — exactly what the lpips package does internally.
"""

import urllib.request
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision.models as tvm
from torchvision.models import AlexNet_Weights, VGG16_Weights


_WEIGHT_URLS = {
    "alex": "https://raw.githubusercontent.com/richzhang/PerceptualSimilarity/master/lpips/weights/v0.1/alex.pth",
    "vgg":  "https://raw.githubusercontent.com/richzhang/PerceptualSimilarity/master/lpips/weights/v0.1/vgg.pth",
}
_CACHE_DIR = Path.home() / ".cache" / "lpips" / "v0.1"

_models: dict[str, "_LPIPS"] = {}


def _fetch_weights(net: str) -> Path:
    _CACHE_DIR.mkdir(parents=True, exist_ok=True)
    path = _CACHE_DIR / f"{net}.pth"
    if not path.exists():
        print(f"Downloading LPIPS {net} weights to {path} ...")
        urllib.request.urlretrieve(_WEIGHT_URLS[net], path)
    return path


class _Extractor(nn.Module):
    _LAYER_IDS = {
        "alex": {1, 4, 7, 9, 11},
        "vgg":  {3, 8, 15, 22, 29},
    }

    def __init__(self, net: str):
        super().__init__()
        if net == "alex":
            self.features = tvm.alexnet(weights=AlexNet_Weights.DEFAULT).features
        else:
            self.features = tvm.vgg16(weights=VGG16_Weights.DEFAULT).features
        self._ids = self._LAYER_IDS[net]
        for p in self.parameters():
            p.requires_grad_(False)

    def forward(self, x: torch.Tensor) -> list[torch.Tensor]:
        out = []
        for i, layer in enumerate(self.features):
            x = layer(x)
            if i in self._ids:
                out.append(x)
        return out


class _LPIPS(nn.Module):
    def __init__(self, net: str):
        super().__init__()
        self.extractor = _Extractor(net)
        weights = torch.load(_fetch_weights(net), map_location="cpu", weights_only=True)
        # weights file: keys lin0.model.1.weight ... lin4.model.1.weight, shape [1, C, 1, 1]
        self.lin_weights = nn.ParameterList([
            nn.Parameter(weights[f"lin{i}.model.1.weight"], requires_grad=False)
            for i in range(5)
        ])
        for p in self.parameters():
            p.requires_grad_(False)

    def forward(self, ref: torch.Tensor, ev: torch.Tensor) -> torch.Tensor:
        ref_feats = self.extractor(ref)
        ev_feats  = self.extractor(ev)
        total = torch.zeros(1)
        for w, rf, ef in zip(self.lin_weights, ref_feats, ev_feats):
            diff = (_channel_norm(rf) - _channel_norm(ef)).pow(2)  # [1, C, H, W]
            total = total + F.conv2d(diff, w).mean()               # weighted spatial mean
        return total


def _channel_norm(feat: torch.Tensor) -> torch.Tensor:
    return feat / feat.norm(dim=1, keepdim=True).clamp(min=1e-10)


def _get_model(net: str) -> _LPIPS:
    if net not in _models:
        _models[net] = _LPIPS(net).eval()
    return _models[net]


def _to_tensor(img: np.ndarray) -> torch.Tensor:
    if img.ndim == 2:
        img = np.stack([img, img, img], axis=2)
    rgb = img[:, :, ::-1].copy()  # BGR -> RGB
    t = torch.from_numpy(rgb).permute(2, 0, 1).float() / 127.5 - 1.0
    return t.unsqueeze(0)


def compute_lpips(ref: np.ndarray, ev: np.ndarray, net: str = "alex") -> float:
    """
    Args:
        ref: reference image as uint8 numpy array (HxWxC BGR or HxW).
        ev:  eval image, same shape/dtype.
        net: 'alex' (default) or 'vgg'.
    Returns:
        LPIPS distance (lower is better).
    """
    model = _get_model(net)
    with torch.no_grad():
        return model(_to_tensor(ref), _to_tensor(ev)).item()
