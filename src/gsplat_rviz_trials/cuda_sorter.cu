#include "gsplat_rviz_trials/cuda_sorter.hpp"

#include <cstdio>

#include <cub/cub.cuh>
#include <cuda_runtime.h>

// ── Helpers ───────────────────────────────────────────────────────────────────

#define CUDA_CHECK(call)                                                        \
  do {                                                                          \
    cudaError_t _e = (call);                                                    \
    if (_e != cudaSuccess) {                                                    \
      fprintf(stderr,                                                           \
        "[gsplat/cuda] %s:%d — %s failed: %s\n",                               \
        __FILE__, __LINE__, #call, cudaGetErrorString(_e));                     \
      return false;                                                             \
    }                                                                           \
  } while (0)

#define CUDA_CHECK_VOID(call)                                                   \
  do {                                                                          \
    cudaError_t _e = (call);                                                    \
    if (_e != cudaSuccess) {                                                    \
      fprintf(stderr,                                                           \
        "[gsplat/cuda] %s:%d — %s failed: %s\n",                               \
        __FILE__, __LINE__, #call, cudaGetErrorString(_e));                     \
    }                                                                           \
  } while (0)

// ── Kernels ───────────────────────────────────────────────────────────────────

__global__ void computeDepthsKernel(
  const float3 * __restrict__ centers,
  float3 fwd,
  float * __restrict__ keys,
  uint32_t n)
{
  const uint32_t i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < n) {
    keys[i] = centers[i].x * fwd.x + centers[i].y * fwd.y + centers[i].z * fwd.z;
  }
}

__global__ void fillFloatIdentityKernel(float * __restrict__ d, uint32_t n)
{
  const uint32_t i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < n) {
    d[i] = static_cast<float>(i);
  }
}

// ── CudaSorter ────────────────────────────────────────────────────────────────

void CudaSorter::init(uint32_t n)
{
  capacity = n;
  cudaStream_t s;
  CUDA_CHECK_VOID(cudaStreamCreate(&s));
  stream_ = s;

  CUDA_CHECK_VOID(cudaMalloc(&d_centers,  n * sizeof(float3)));
  CUDA_CHECK_VOID(cudaMalloc(&d_keys_in,  n * sizeof(float)));
  CUDA_CHECK_VOID(cudaMalloc(&d_keys_out, n * sizeof(float)));
  CUDA_CHECK_VOID(cudaMalloc(&d_vals_in,  n * sizeof(float)));
  CUDA_CHECK_VOID(cudaMalloc(&d_vals_out, n * sizeof(float)));
  CUDA_CHECK_VOID(cudaMallocHost(&h_vals_out, n * sizeof(float)));

  // Fill identity permutation once — d_vals_in[i] = (float)i, never modified.
  fillFloatIdentityKernel<<<(n + 255) / 256, 256, 0, s>>>(
    reinterpret_cast<float *>(d_vals_in), n);
  CUDA_CHECK_VOID(cudaGetLastError());

  // Determine CUB temporary storage size.
  CUDA_CHECK_VOID(cub::DeviceRadixSort::SortPairsDescending(
    nullptr, temp_bytes,
    reinterpret_cast<float *>(d_keys_in),
    reinterpret_cast<float *>(d_keys_out),
    reinterpret_cast<float *>(d_vals_in),
    reinterpret_cast<float *>(d_vals_out),
    static_cast<int>(n), 0, 32, s));
  CUDA_CHECK_VOID(cudaMalloc(&d_temp, temp_bytes));

  CUDA_CHECK_VOID(cudaStreamSynchronize(s));
}

void CudaSorter::destroy()
{
  if (h_vals_out) { cudaFreeHost(h_vals_out); h_vals_out = nullptr; }
  if (d_centers)  { cudaFree(d_centers);  d_centers  = nullptr; }
  if (d_keys_in)  { cudaFree(d_keys_in);  d_keys_in  = nullptr; }
  if (d_keys_out) { cudaFree(d_keys_out); d_keys_out = nullptr; }
  if (d_vals_in)  { cudaFree(d_vals_in);  d_vals_in  = nullptr; }
  if (d_vals_out) { cudaFree(d_vals_out); d_vals_out = nullptr; }
  if (d_temp)     { cudaFree(d_temp);     d_temp     = nullptr; }
  if (stream_) {
    cudaStreamDestroy(reinterpret_cast<cudaStream_t>(stream_));
    stream_ = nullptr;
  }
  capacity   = 0;
  temp_bytes = 0;
}

void CudaSorter::uploadCenters(const float * xyz, uint32_t n)
{
  CUDA_CHECK_VOID(cudaMemcpyAsync(
    d_centers, xyz, n * sizeof(float3),
    cudaMemcpyHostToDevice,
    reinterpret_cast<cudaStream_t>(stream_)));
}

bool CudaSorter::sort(const float cam_fwd[3], uint32_t n)
{
  auto s = reinterpret_cast<cudaStream_t>(stream_);

  // 1. Compute per-splat depth keys.
  const float3 fwd{cam_fwd[0], cam_fwd[1], cam_fwd[2]};
  computeDepthsKernel<<<(n + 255) / 256, 256, 0, s>>>(
    reinterpret_cast<const float3 *>(d_centers),
    fwd,
    reinterpret_cast<float *>(d_keys_in),
    n);
  CUDA_CHECK(cudaGetLastError());

  // 2. CUB descending sort: d_vals_in (identity) → d_vals_out (sorted indices).
  CUDA_CHECK(cub::DeviceRadixSort::SortPairsDescending(
    d_temp, temp_bytes,
    reinterpret_cast<float *>(d_keys_in),
    reinterpret_cast<float *>(d_keys_out),
    reinterpret_cast<float *>(d_vals_in),
    reinterpret_cast<float *>(d_vals_out),
    static_cast<int>(n), 0, 32, s));

  // 3. Copy sorted indices device→host (pinned, so DMA is fast).
  CUDA_CHECK(cudaMemcpyAsync(
    h_vals_out, d_vals_out, n * sizeof(float),
    cudaMemcpyDeviceToHost, s));

  // 4. Wait for the copy to complete before the CPU reads h_vals_out.
  CUDA_CHECK(cudaStreamSynchronize(s));

  return true;
}
