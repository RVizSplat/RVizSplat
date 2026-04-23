#include "gsplat_rviz_trials/sorters/cuda_splat_sorter.hpp"

#include <algorithm>
#include <vector>

namespace gsplat_rviz_trials
{

void CudaSplatSorter::uploadCenters(const std::vector<Ogre::Vector3> & centers)
{
  backend_.destroy();
  count_        = static_cast<uint32_t>(centers.size());
  result_ready_ = false;
  failed_       = false;
  if (count_ == 0) return;

  backend_.init(count_);
  backend_.uploadCenters(&centers[0].x, count_);
}

void CudaSplatSorter::requestSort(const Ogre::Vector3 & cam_fwd)
{
  if (count_ == 0 || failed_) return;

  const float fwd[3] = {cam_fwd.x, cam_fwd.y, cam_fwd.z};
  if (!backend_.sort(fwd, count_)) {
    failed_ = true;
    return;
  }
  result_ready_ = true;
}

std::unique_ptr<SortResult> CudaSplatSorter::pollResult()
{
  if (!result_ready_) return nullptr;
  result_ready_ = false;

  auto r = std::make_unique<SortResult>();
  r->count = count_;
  r->indices.resize(count_);
  const float * src = static_cast<const float *>(backend_.h_vals_out);
  std::copy(src, src + count_, r->indices.begin());
  return r;
}

void CudaSplatSorter::reset()
{
  backend_.destroy();
  count_        = 0;
  result_ready_ = false;
  failed_       = false;
}

}  // namespace gsplat_rviz_trials
