#ifndef GSPLAT_RVIZ_TRIALS__CPU_SORTER_HPP_
#define GSPLAT_RVIZ_TRIALS__CPU_SORTER_HPP_

#include <cstdint>
#include <vector>

#include <OgreVector3.h>

#include "gsplat_rviz_trials/sorters/i_splat_sorter.hpp"

namespace gsplat_rviz_trials
{

// Synchronous CPU depth-sort (pdqsort). No threading — whichever thread the
// caller invokes sort() from runs the work.
class CpuSorter : public ISplatSorter
{
public:
  CpuSorter() = default;
  ~CpuSorter() override = default;

  void uploadCenters(const std::vector<Ogre::Vector3> & centers) override;
  SortResult sort(const Ogre::Vector3 & cam_fwd) override;
  const char * name() const override { return "CPU"; }

private:
  std::vector<Ogre::Vector3> centers_;
  std::vector<float>         depth_keys_;
  std::vector<uint32_t>      sort_indices_;  // retained across frames → near-sorted input
};

}  // namespace gsplat_rviz_trials

#endif  // GSPLAT_RVIZ_TRIALS__CPU_SORTER_HPP_
