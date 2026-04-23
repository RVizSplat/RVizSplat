#ifndef GSPLAT_RVIZ_TRIALS__CPU_SORTER_HPP_
#define GSPLAT_RVIZ_TRIALS__CPU_SORTER_HPP_

#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <OgreVector3.h>

#include "gsplat_rviz_trials/sorters/i_splat_sorter.hpp"

namespace gsplat_rviz_trials
{

// Background-thread pdqsort; one-frame latency.
class CpuSorter : public ISplatSorter
{
public:
  CpuSorter();
  ~CpuSorter() override;

  void uploadCenters(const std::vector<Ogre::Vector3> & centers) override;
  void requestSort(const Ogre::Vector3 & cam_fwd) override;
  std::unique_ptr<SortResult> pollResult() override;
  void reset() override;
  const char * name() const override { return "CPU"; }

private:
  void workerMain();
  void waitForIdle();

  std::vector<Ogre::Vector3> centers_;
  std::vector<float>         depth_keys_;
  std::vector<uint32_t>      sort_indices_;  // retained across frames → near-sorted input
  std::vector<float>         back_buf_;      // worker writes here
  std::vector<float>         front_buf_;     // completed result waiting to be polled
  uint32_t                   front_count_ = 0;

  std::thread             thread_;
  std::mutex              mutex_;
  std::condition_variable wake_cv_;
  std::condition_variable idle_cv_;
  bool          shutdown_        = false;
  bool          request_pending_ = false;
  bool          running_         = false;
  bool          result_ready_    = false;
  Ogre::Vector3 req_fwd_;
};

}  // namespace gsplat_rviz_trials

#endif  // GSPLAT_RVIZ_TRIALS__CPU_SORTER_HPP_
