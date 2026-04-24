#include "gsplat_rviz_plugin/sorters/sorter_factory.hpp"

#include <rclcpp/logging.hpp>

#include "gsplat_rviz_plugin/sorters/cpu_sorter.hpp"

#ifdef GSPLAT_USE_CUDA
#include <cuda_runtime.h>
#include "gsplat_rviz_plugin/sorters/cuda_splat_sorter.hpp"
#endif

namespace gsplat_rviz_plugin
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
  if (kind == SorterKind::Cuda) {
#ifdef GSPLAT_USE_CUDA
    if (cudaAvailable()) {
      return std::make_unique<CudaSplatSorter>();
    }
    RCLCPP_WARN(
      rclcpp::get_logger("gsplat_rviz_plugin"),
      "CUDA sort requested but no CUDA device is available — falling back to CPU sort.");
#else
    RCLCPP_WARN(
      rclcpp::get_logger("gsplat_rviz_plugin"),
      "CUDA sort requested but this build has no CUDA support — falling back to CPU sort.");
#endif
  }
  return std::make_unique<CpuSorter>();
}

}  // namespace gsplat_rviz_plugin
