#include "gsplat_rviz_trials/sorters/sorter_factory.hpp"

#include <rclcpp/logging.hpp>

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
  if (want_cuda && cudaAvailable()) {
    return std::make_unique<CudaSplatSorter>();
  }
  if (kind == SorterKind::Cuda) {
    RCLCPP_ERROR(
      rclcpp::get_logger("gsplat_rviz_trials"),
      "CUDA sort requested but no CUDA device is available — falling back to CPU sort.");
  }
#else
  if (kind == SorterKind::Cuda) {
    RCLCPP_ERROR(
      rclcpp::get_logger("gsplat_rviz_trials"),
      "CUDA sort requested but this build has no CUDA support — falling back to CPU sort.");
  }
#endif
  return std::make_unique<CpuSorter>();
}

}  // namespace gsplat_rviz_trials
