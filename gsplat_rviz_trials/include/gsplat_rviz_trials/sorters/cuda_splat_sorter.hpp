#ifndef GSPLAT_RVIZ_TRIALS__CUDA_SPLAT_SORTER_HPP_
#define GSPLAT_RVIZ_TRIALS__CUDA_SPLAT_SORTER_HPP_

#include <memory>
#include <vector>

#include <OgreVector3.h>

#include "gsplat_rviz_trials/sorters/i_splat_sorter.hpp"

namespace gsplat_rviz_trials
{

// Synchronous CUB-based GPU depth-sort. No threading — sort() blocks on
// cudaStreamSynchronize on the caller's thread. For CUDA-context safety
// callers should invoke all methods from a single thread.
class CudaSplatSorter : public ISplatSorter
{
public:
  CudaSplatSorter();
  ~CudaSplatSorter() override;

  void uploadCenters(const std::vector<Ogre::Vector3> & centers) override;
  SortResult sort(const Ogre::Vector3 & cam_fwd) override;
  const char * name() const override { return "CUDA"; }

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gsplat_rviz_trials

#endif  // GSPLAT_RVIZ_TRIALS__CUDA_SPLAT_SORTER_HPP_
