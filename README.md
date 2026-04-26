# RVizSplat

## Evaluation

The `evaluation/eval.py` script computes image quality metrics (PSNR, SSIM, LPIPS) between a ref_folder and an eval_folder.

Images are matched by the trailing 3-digit number in the filename (e.g. `img_001.png` in the ref_folder is paired with `*_001.png` in the eval_folder).

### Usage

```bash
cd evaluation
python eval.py <ref_folder> <eval_folder> [--metrics psnr ssim lpips] [--lpips-net alex|vgg]
```

| Argument | Description |
|---|---|
| `ref_folder` | Folder containing reference (ground-truth) images |
| `eval_folder` | Folder containing images to evaluate |
| `--metrics` | Space-separated list of metrics to compute (default: all three) |
| `--lpips-net` | Backbone network for LPIPS — `alex` (default) or `vgg` |

### Examples

Compute all metrics using the default AlexNet backbone:
```bash
python eval.py data/ref data/eval
```

Compute PSNR and LPIPS with VGG backbone:
```bash
python eval.py data/ref data/eval --metrics psnr lpips --lpips-net vgg
```

### Output

The script prints a per-image table and a mean row at the bottom:

```
Image                    PSNR        SSIM       LPIPS
--------------------------------------------------------
img_001.png           32.1500      0.9210      0.0431
img_002.png           29.8300      0.8970      0.0612
--------------------------------------------------------
Mean                  30.9900      0.9090      0.0522
```