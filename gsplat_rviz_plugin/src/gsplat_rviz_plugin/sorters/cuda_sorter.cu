#include "gsplat_rviz_plugin/sorters/cuda_splat_sorter.hpp"

#include <algorithm>
#include <cstdio>

#include <cub/cub.cuh>
#include <cuda_runtime.h>

#include "gsplat_rviz_plugin/perf_monitor.hpp"

namespace gsplat_rviz_plugin
{

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

namespace {

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

}  // namespace

// ── Impl ──────────────────────────────────────────────────────────────────────

struct CudaSplatSorter::Impl
{
  float3 *     d_centers  = nullptr;
  float *      d_keys_in  = nullptr;
  float *      d_keys_out = nullptr;
  float *      d_vals_in  = nullptr;  // identity permutation (set once)
  float *      d_vals_out = nullptr;
  float *      h_vals_out = nullptr;  // pinned host
  void *       d_temp     = nullptr;
  size_t       temp_bytes = 0;
  uint32_t     capacity   = 0;
  uint32_t     count      = 0;
  cudaStream_t stream     = nullptr;
  bool         result_ready = false;
  bool         failed       = false;

  void allocate(uint32_t n);
  void release();
  void upload(const float * xyz, uint32_t n);
  bool sort(const float cam_fwd[3], uint32_t n);
};

void CudaSplatSorter::Impl::allocate(uint32_t n)
{
  capacity = n;
  CUDA_CHECK_VOID(cudaStreamCreate(&stream));

  CUDA_CHECK_VOID(cudaMalloc(&d_centers,  n * sizeof(float3)));
  CUDA_CHECK_VOID(cudaMalloc(&d_keys_in,  n * sizeof(float)));
  CUDA_CHECK_VOID(cudaMalloc(&d_keys_out, n * sizeof(float)));
  CUDA_CHECK_VOID(cudaMalloc(&d_vals_in,  n * sizeof(float)));
  CUDA_CHECK_VOID(cudaMalloc(&d_vals_out, n * sizeof(float)));
  CUDA_CHECK_VOID(cudaMallocHost(&h_vals_out, n * sizeof(float)));

  fillFloatIdentityKernel<<<(n + 255) / 256, 256, 0, stream>>>(d_vals_in, n);
  CUDA_CHECK_VOID(cudaGetLastError());

  CUDA_CHECK_VOID(cub::DeviceRadixSort::SortPairsDescending(
    nullptr, temp_bytes,
    d_keys_in, d_keys_out, d_vals_in, d_vals_out,
    static_cast<int>(n), 0, 32, stream));
  CUDA_CHECK_VOID(cudaMalloc(&d_temp, temp_bytes));

  CUDA_CHECK_VOID(cudaStreamSynchronize(stream));
}

void CudaSplatSorter::Impl::release()
{
  if (h_vals_out) { cudaFreeHost(h_vals_out); h_vals_out = nullptr; }
  if (d_centers)  { cudaFree(d_centers);  d_centers  = nullptr; }
  if (d_keys_in)  { cudaFree(d_keys_in);  d_keys_in  = nullptr; }
  if (d_keys_out) { cudaFree(d_keys_out); d_keys_out = nullptr; }
  if (d_vals_in)  { cudaFree(d_vals_in);  d_vals_in  = nullptr; }
  if (d_vals_out) { cudaFree(d_vals_out); d_vals_out = nullptr; }
  if (d_temp)     { cudaFree(d_temp);     d_temp     = nullptr; }
  if (stream) {
    cudaStreamDestroy(stream);
    stream = nullptr;
  }
  capacity     = 0;
  count        = 0;
  temp_bytes   = 0;
  result_ready = false;
  failed       = false;
}

void CudaSplatSorter::Impl::upload(const float * xyz, uint32_t n)
{
  CUDA_CHECK_VOID(cudaMemcpyAsync(
    d_centers, xyz, n * sizeof(float3),
    cudaMemcpyHostToDevice, stream));
}

bool CudaSplatSorter::Impl::sort(const float cam_fwd[3], uint32_t n)
{
  const float3 fwd{cam_fwd[0], cam_fwd[1], cam_fwd[2]};
  computeDepthsKernel<<<(n + 255) / 256, 256, 0, stream>>>(
    d_centers, fwd, d_keys_in, n);
  CUDA_CHECK(cudaGetLastError());

  CUDA_CHECK(cub::DeviceRadixSort::SortPairsDescending(
    d_temp, temp_bytes,
    d_keys_in, d_keys_out, d_vals_in, d_vals_out,
    static_cast<int>(n), 0, 32, stream));

  CUDA_CHECK(cudaMemcpyAsync(
    h_vals_out, d_vals_out, n * sizeof(float),
    cudaMemcpyDeviceToHost, stream));

  CUDA_CHECK(cudaStreamSynchronize(stream));
  return true;
}

// ── CudaSplatSorter ───────────────────────────────────────────────────────────

CudaSplatSorter::CudaSplatSorter() : impl_(std::make_unique<Impl>()) {}

CudaSplatSorter::~CudaSplatSorter() { impl_->release(); }

void CudaSplatSorter::uploadCenters(const std::vector<Ogre::Vector3> & centers)
{
  impl_->release();
  const uint32_t n = static_cast<uint32_t>(centers.size());
  if (n == 0) return;

  PerfMonitor::instance().startTimer("cuda_upload");
  impl_->count = n;
  impl_->allocate(n);
  impl_->upload(&centers[0].x, n);
  CUDA_CHECK_VOID(cudaStreamSynchronize(impl_->stream));
  PerfMonitor::instance().stopTimer("cuda_upload");
}

SortResult CudaSplatSorter::sort(const Ogre::Vector3 & cam_fwd)
{
  if (impl_->count == 0 || impl_->failed) return {};

  const float fwd[3] = {cam_fwd.x, cam_fwd.y, cam_fwd.z};
  PerfMonitor::instance().startTimer("cuda_sort");
  const bool ok = impl_->sort(fwd, impl_->count);
  PerfMonitor::instance().stopTimer("cuda_sort");
  if (!ok) {
    impl_->failed = true;
    return {};
  }

  SortResult r;
  r.count = impl_->count;
  r.indices.resize(impl_->count);
  std::copy(impl_->h_vals_out, impl_->h_vals_out + impl_->count, r.indices.begin());
  return r;
}

}  // namespace gsplat_rviz_plugin
