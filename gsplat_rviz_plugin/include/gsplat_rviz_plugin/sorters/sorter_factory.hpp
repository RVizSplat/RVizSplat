#ifndef GSPLAT_RVIZ_PLUGIN__SORTER_FACTORY_HPP_
#define GSPLAT_RVIZ_PLUGIN__SORTER_FACTORY_HPP_

#include <memory>

#include "gsplat_rviz_plugin/sorters/i_splat_sorter.hpp"

namespace gsplat_rviz_plugin
{

enum class SorterKind
{
  Cpu  = 0,
  Cuda = 1,   // falls back to CPU if no device / no CUDA build
};

// Create a sorter of the requested kind. Falls back to CPU if CUDA was
// requested but no device is available or the build was made without CUDA.
std::unique_ptr<ISplatSorter> makeSorter(SorterKind kind);

// True if the build includes the CUDA backend AND a CUDA device is detected.
bool cudaAvailable();

}  // namespace gsplat_rviz_plugin

#endif  // GSPLAT_RVIZ_PLUGIN__SORTER_FACTORY_HPP_
