#include "gsplat_rviz_trials/sorters/cpu_sorter.hpp"

#include <numeric>

#include <boost/sort/sort.hpp>

#include "gsplat_rviz_trials/perf_monitor.hpp"

namespace gsplat_rviz_trials
{

void CpuSorter::uploadCenters(const std::vector<Ogre::Vector3> & centers)
{
  centers_ = centers;
  depth_keys_.assign(centers_.size(), 0.0f);
  sort_indices_.resize(centers_.size());
  std::iota(sort_indices_.begin(), sort_indices_.end(), 0u);
}

SortResult CpuSorter::sort(const Ogre::Vector3 & cam_fwd)
{
  const uint32_t count = static_cast<uint32_t>(centers_.size());
  if (count == 0) return {};

  PerfMonitor::instance().startTimer("cpu_sort");

  for (uint32_t i = 0; i < count; ++i) {
    depth_keys_[i] = cam_fwd.dotProduct(centers_[i]);
  }

  // sort_indices_ retains the previous frame's order so pdqsort only fixes
  // the inversions caused by camera movement (near O(n) for small drags).
  boost::sort::pdqsort(
    sort_indices_.begin(), sort_indices_.end(),
    [this](uint32_t a, uint32_t b) {
      return depth_keys_[a] > depth_keys_[b];  // descending = back-to-front
    });

  SortResult r;
  r.count = count;
  r.indices.resize(count);
  for (uint32_t i = 0; i < count; ++i) {
    r.indices[i] = static_cast<float>(sort_indices_[i]);
  }

  PerfMonitor::instance().stopTimer("cpu_sort");
  return r;
}

}  // namespace gsplat_rviz_trials
