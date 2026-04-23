#ifndef GSPLAT_RVIZ_TRIALS__SORTER_FACTORY_HPP_
#define GSPLAT_RVIZ_TRIALS__SORTER_FACTORY_HPP_

#include <memory>

#include "gsplat_rviz_trials/sorters/i_splat_sorter.hpp"

namespace gsplat_rviz_trials
{

enum class SorterKind
{
  Auto = 0,   // CUDA if available at runtime, else CPU
  Cpu  = 1,
  Cuda = 2,
};

// Create a sorter of the requested kind. Falls back to CPU if CUDA was
// requested but no device is available or the build was made without CUDA.
std::unique_ptr<ISplatSorter> makeSorter(SorterKind kind);

// True if the build includes the CUDA backend AND a CUDA device is detected.
bool cudaAvailable();

}  // namespace gsplat_rviz_trials

#endif  // GSPLAT_RVIZ_TRIALS__SORTER_FACTORY_HPP_
