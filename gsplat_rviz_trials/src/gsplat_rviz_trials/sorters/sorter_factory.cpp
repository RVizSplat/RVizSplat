#include "gsplat_rviz_trials/sorters/sorter_factory.hpp"

#include "gsplat_rviz_trials/sorters/cpu_sorter.hpp"

#ifdef GSPLAT_USE_CUDA
#include <cuda_runtime.h>
#include "gsplat_rviz_trials/sorters/cuda_splat_sorter.hpp"
#endif

namespace gsplat_rviz_trials
{

bool cudaAvailable()
{
#ifdef GSPLAT_USE_CUDA
  int n = 0;
  return cudaGetDeviceCount(&n) == cudaSuccess && n > 0;
#else
  return false;
#endif
}

std::unique_ptr<ISplatSorter> makeSorter(SorterKind kind)
{
#ifdef GSPLAT_USE_CUDA
  const bool want_cuda = (kind == SorterKind::Cuda) ||
                         (kind == SorterKind::Auto && cudaAvailable());
  if (want_cuda && cudaAvailable()) 
  {
    return std::make_unique<CudaSplatSorter>();
  }
#else
  (void)kind;
#endif
  return std::make_unique<CpuSorter>();
}

}  // namespace gsplat_rviz_trials
