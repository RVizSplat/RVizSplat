# RVizSplat

RVizSplat is an RViz2 display plugin that provides end-to-end visualization of 3D Gaussian Splats in RViz.

<p align="center">
  <img src="images/rviz_splat.jpg" alt="RVizSplat" width="400">
</p>

### Build status

[![Rolling](https://github.com/RVizSplat/RVizSplat/actions/workflows/rolling.yml/badge.svg)](https://github.com/RVizSplat/RVizSplat/actions/workflows/rolling.yml) &nbsp;&nbsp;&nbsp;
[![Kilted](https://github.com/RVizSplat/RVizSplat/actions/workflows/kilted.yml/badge.svg)](https://github.com/RVizSplat/RVizSplat/actions/workflows/kilted.yml) &nbsp;&nbsp;&nbsp;
[![Jazzy](https://github.com/RVizSplat/RVizSplat/actions/workflows/jazzy.yml/badge.svg)](https://github.com/RVizSplat/RVizSplat/actions/workflows/jazzy.yml)

# How to build and run from source

```bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
git clone https://github.com/RVizSplat/RVizSplat.git
cd ~/ros_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

# Installation via apt

This feature is currently under development (See https://github.com/ros/rosdistro/pull/50909 for details)

After sourcing your ROS 2 environment:

```bash
sudo apt-get install ros-$ROS_DISTRO-gsplat-rviz-plugin ros-$ROS_DISTRO-gsplat-publisher ros-$ROS_DISTRO-gsplat-msgs
```

# Examples

# Using OIT for performance optimization

# Architecture

# Evaluation

The `gsplat_plugin_evaluation/eval.py` script computes image quality metrics (PSNR, SSIM, LPIPS) between a ref_folder and an eval_folder.

Images are matched by the trailing 3-digit number in the filename (e.g. `img_001.png` in the ref_folder is paired with `*_001.png` in the eval_folder).

### Usage

```bash
cd gsplat_plugin_evaluation
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