#!/usr/bin/env python3
"""Monitor CPU, RAM, GPU, and GPU memory usage of a process by PID.

Usage:
    python monitor_process.py <PID> [--interval 1.0] [--output stats.csv] [--duration 60]

Requires: psutil. Optional: pynvml (for NVIDIA GPU metrics).
    pip install psutil nvidia-ml-py
"""

import argparse
import csv
import signal
import sys
import time
from datetime import datetime

import psutil

try:
    import pynvml
    pynvml.nvmlInit()
    NVML_AVAILABLE = True
    NUM_GPUS = pynvml.nvmlDeviceGetCount()
except Exception as e:
    NVML_AVAILABLE = False
    NUM_GPUS = 0
    print(f"[warn] NVML unavailable, GPU metrics disabled: {e}", file=sys.stderr)


def get_gpu_stats(pid):
    """Return (gpu_util_percent, gpu_mem_bytes) summed across all GPUs for pid.

    gpu_util_percent is the SM utilization sample reported by NVML for this
    process (sum across GPUs if the process spans multiple). Returns
    (None, None) when NVML is unavailable, or (0.0, 0) when the pid is not
    using any GPU.
    """
    if not NVML_AVAILABLE:
        return None, None
    total_mem = 0
    total_util = 0.0
    found = False
    for i in range(NUM_GPUS):
        handle = pynvml.nvmlDeviceGetHandleByIndex(i)
        for getter in (
            pynvml.nvmlDeviceGetComputeRunningProcesses,
            pynvml.nvmlDeviceGetGraphicsRunningProcesses,
        ):
            try:
                for p in getter(handle):
                    if p.pid == pid and p.usedGpuMemory is not None:
                        total_mem += p.usedGpuMemory
                        found = True
            except pynvml.NVMLError:
                pass
        try:
            for u in pynvml.nvmlDeviceGetProcessUtilization(handle, 0):
                if u.pid == pid:
                    total_util += u.smUtil
                    found = True
        except pynvml.NVMLError:
            # No recent samples for this device.
            pass
    if not found:
        return 0.0, 0
    return total_util, total_mem


def main():
    parser = argparse.ArgumentParser(
        description="Sample CPU/RAM/GPU usage of a process and write to CSV."
    )
    parser.add_argument("pid", type=int, help="PID of the process to monitor.")
    parser.add_argument(
        "--interval", type=float, default=1.0,
        help="Sampling interval in seconds (default: 1.0).",
    )
    parser.add_argument(
        "--output", "-o", default=None,
        help="Output CSV path (default: proc_<pid>_usage_<timestamp>.csv).",
    )
    parser.add_argument(
        "--duration", type=float, default=None,
        help="Optional total duration in seconds; otherwise runs until the "
             "process exits or Ctrl+C.",
    )
    args = parser.parse_args()

    try:
        proc = psutil.Process(args.pid)
    except psutil.NoSuchProcess:
        print(f"No process with PID {args.pid}", file=sys.stderr)
        sys.exit(1)

    output = args.output or (
        f"proc_{args.pid}_usage_{datetime.now():%Y%m%d_%H%M%S}.csv"
    )

    # Prime cpu_percent so the first real reading isn't 0.0.
    proc.cpu_percent(interval=None)

    fields = [
        "timestamp",
        "elapsed_s",
        "cpu_percent",
        "ram_rss_mb",
        "ram_percent",
        "gpu_util_percent",
        "gpu_mem_mb",
    ]

    stop = False

    def handle_sig(signum, frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, handle_sig)
    signal.signal(signal.SIGTERM, handle_sig)

    start = time.time()
    print(
        f"Monitoring PID {args.pid} every {args.interval}s -> {output}. "
        f"Ctrl+C to stop."
    )

    try:
        with open(output, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            while not stop:
                if not proc.is_running() or proc.status() == psutil.STATUS_ZOMBIE:
                    print(f"Process {args.pid} is no longer running. Stopping.")
                    break
                tick = time.time()
                try:
                    cpu = proc.cpu_percent(interval=None)
                    rss_mb = proc.memory_info().rss / (1024 * 1024)
                    ram_pct = proc.memory_percent()
                except psutil.NoSuchProcess:
                    print(f"Process {args.pid} exited. Stopping.")
                    break

                gpu_util, gpu_mem = get_gpu_stats(args.pid)
                gpu_mem_mb = gpu_mem / (1024 * 1024) if gpu_mem is not None else None

                writer.writerow({
                    "timestamp": datetime.now().isoformat(timespec="milliseconds"),
                    "elapsed_s": round(tick - start, 3),
                    "cpu_percent": round(cpu, 2),
                    "ram_rss_mb": round(rss_mb, 2),
                    "ram_percent": round(ram_pct, 2),
                    "gpu_util_percent": "" if gpu_util is None else round(gpu_util, 2),
                    "gpu_mem_mb": "" if gpu_mem_mb is None else round(gpu_mem_mb, 2),
                })
                f.flush()

                if args.duration is not None and (tick - start) >= args.duration:
                    break

                # Compensate for sampling cost so we stay close to `interval`.
                time.sleep(max(0.0, args.interval - (time.time() - tick)))
    finally:
        if NVML_AVAILABLE:
            try:
                pynvml.nvmlShutdown()
            except Exception:
                pass

    print(f"Saved {output}")


if __name__ == "__main__":
    main()
