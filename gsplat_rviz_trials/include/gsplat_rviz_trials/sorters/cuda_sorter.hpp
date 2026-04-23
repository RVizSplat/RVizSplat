#pragma once
#include <cstddef>
#include <cstdint>

// Plain C++ header — no CUDA types, safe to include from any translation unit.
struct CudaSorter
{
  void *   d_centers  = nullptr;  // float3[N] splat centres on device
  void *   d_keys_in  = nullptr;  // float[N]  depth keys (written each frame)
  void *   d_keys_out = nullptr;  // float[N]  CUB key scratch
  void *   d_vals_in  = nullptr;  // float[N]  identity [0,1,…,N-1] (read-only)
  void *   d_vals_out = nullptr;  // float[N]  sorted indices on device
  void *   h_vals_out = nullptr;  // float[N]  sorted indices on host (pinned)
  void *   d_temp     = nullptr;  // CUB temporary storage
  size_t   temp_bytes = 0;
  uint32_t capacity   = 0;
  void *   stream_    = nullptr;  // opaque cudaStream_t

  void init(uint32_t n);
  void destroy();

  // Upload splat centres to device once per setSplats().
  void uploadCenters(const float * xyz_stride3, uint32_t n);

  // Sort splats back-to-front; result is in h_vals_out (pinned host memory).
  // Returns false if any CUDA call fails (caller should fall back to CPU sort).
  bool sort(const float cam_fwd[3], uint32_t n);
};
