#ifndef GSPLAT_RVIZ_TRIALS__CUDA_SPLAT_SORTER_HPP_
#define GSPLAT_RVIZ_TRIALS__CUDA_SPLAT_SORTER_HPP_

#include <memory>
#include <vector>

#include <OgreVector3.h>

#include "gsplat_rviz_trials/sorters/cuda_sorter.hpp"
#include "gsplat_rviz_trials/sorters/i_splat_sorter.hpp"

namespace gsplat_rviz_trials
{

// ISplatSorter adapter over the raw CUDA backend. Synchronous: sort runs
// inline in requestSort() (~1–2 ms for 1M splats), result ready same frame.
class CudaSplatSorter : public ISplatSorter
{
public:
  CudaSplatSorter() = default;
  ~CudaSplatSorter() override { backend_.destroy(); }

  void uploadCenters(const std::vector<Ogre::Vector3> & centers) override;
  void requestSort(const Ogre::Vector3 & cam_fwd) override;
  std::unique_ptr<SortResult> pollResult() override;
  void reset() override;
  const char * name() const override { return "CUDA"; }

private:
  CudaSorter backend_;
  uint32_t   count_ = 0;
  bool       result_ready_ = false;
  bool       failed_ = false;
};

}  // namespace gsplat_rviz_trials

#endif  // GSPLAT_RVIZ_TRIALS__CUDA_SPLAT_SORTER_HPP_
