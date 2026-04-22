#pragma once
#include <cstddef>
#include <cstdint>

// Plain C++ header — no CUDA types, safe to include from any translation unit.
// CudaSorter is only instantiated when GSPLAT_USE_CUDA is defined.
struct CudaSorter
{
  void *   d_centers  = nullptr;  // float3[N]  splat centres on device
  void *   d_keys_in  = nullptr;  // float[N]   depth keys (written each frame)
  void *   d_keys_out = nullptr;  // float[N]   CUB key scratch
  void *   d_vals_in  = nullptr;  // float[N]   identity [0,1,…,N-1] (read-only)
  void *   d_vals_out = nullptr;  // float[N]   CUB sorted output on device
  void *   h_vals_out = nullptr;  // float[N]   sorted output on host (pinned DMA)
  void *   d_temp     = nullptr;  // CUB temporary storage
  size_t   temp_bytes = 0;
  uint32_t capacity   = 0;
  void *   stream_    = nullptr;  // opaque cudaStream_t

  void init(uint32_t n);
  void destroy();

  // Upload splat centres to device (call once after setSplats fills centers_).
  // xyz_stride3: contiguous float3 array of n entries (layout-compatible with Ogre::Vector3[n]).
  void uploadCenters(const float * xyz_stride3, uint32_t n);

  // Sort splat indices back-to-front; result is written into h_vals_out (pinned host).
  // Returns false if any CUDA call fails (caller falls back to CPU sort).
  bool sort(const float cam_fwd[3], uint32_t n);
};
