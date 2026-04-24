#include "gsplat_rviz_trials/sorters/cpu_sorter.hpp"

#include <numeric>

#include <boost/sort/sort.hpp>

#include "gsplat_rviz_trials/perf_monitor.hpp"

namespace gsplat_rviz_trials
{

CpuSorter::CpuSorter()
{
  thread_ = std::thread(&CpuSorter::workerMain, this);
}

CpuSorter::~CpuSorter()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    shutdown_        = true;
    request_pending_ = false;
  }
  wake_cv_.notify_all();
  if (thread_.joinable()) thread_.join();
}

void CpuSorter::waitForIdle()
{
  std::unique_lock<std::mutex> lock(mutex_);
  request_pending_ = false;
  idle_cv_.wait(lock, [this] { return !running_; });
}

void CpuSorter::uploadCenters(const std::vector<Ogre::Vector3> & centers)
{
  waitForIdle();

  centers_      = centers;
  depth_keys_.assign(centers_.size(), 0.0f);
  sort_indices_.resize(centers_.size());
  std::iota(sort_indices_.begin(), sort_indices_.end(), 0u);
  back_buf_.assign(centers_.size(), 0.0f);

  std::lock_guard<std::mutex> lock(mutex_);
  result_ready_ = false;
  front_buf_.clear();
  front_count_ = 0;
}

void CpuSorter::requestSort(const Ogre::Vector3 & cam_fwd)
{
  if (centers_.empty()) return;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    req_fwd_         = cam_fwd;
    request_pending_ = true;
  }
  wake_cv_.notify_one();
}

std::unique_ptr<SortResult> CpuSorter::pollResult()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!result_ready_) return nullptr;

  auto r = std::make_unique<SortResult>();
  r->indices = std::move(front_buf_);
  r->count   = front_count_;
  front_buf_.clear();
  front_count_  = 0;
  result_ready_ = false;
  return r;
}

void CpuSorter::reset()
{
  waitForIdle();

  centers_.clear();
  depth_keys_.clear();
  sort_indices_.clear();
  back_buf_.clear();

  std::lock_guard<std::mutex> lock(mutex_);
  result_ready_ = false;
  front_buf_.clear();
  front_count_ = 0;
}

void CpuSorter::workerMain()
{
  while (true) {
    Ogre::Vector3 fwd;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      wake_cv_.wait(lock, [this] { return shutdown_ || request_pending_; });
      if (shutdown_) return;

      fwd              = req_fwd_;
      request_pending_ = false;
      running_         = true;
    }

    const uint32_t count = static_cast<uint32_t>(centers_.size());
    if (count > 0 &&
        depth_keys_.size() == count &&
        sort_indices_.size() == count)
    {
      PerfMonitor::instance().startTimer("cpu_sort");

      for (uint32_t i = 0; i < count; ++i) {
        depth_keys_[i] = fwd.dotProduct(centers_[i]);
      }

      // sort_indices_ retains the previous frame's order so pdqsort only fixes
      // the inversions caused by camera movement (near O(n) for small drags).
      boost::sort::pdqsort(
        sort_indices_.begin(), sort_indices_.end(),
        [this](uint32_t a, uint32_t b) {
          return depth_keys_[a] > depth_keys_[b];  // descending = back-to-front
        });

      if (back_buf_.size() != count) back_buf_.resize(count);
      for (uint32_t i = 0; i < count; ++i) {
        back_buf_[i] = static_cast<float>(sort_indices_[i]);
      }

      PerfMonitor::instance().stopTimer("cpu_sort");

      {
        std::lock_guard<std::mutex> lock(mutex_);
        std::swap(front_buf_, back_buf_);
        front_count_  = count;
        result_ready_ = true;
        running_      = false;
      }
    } else {
      std::lock_guard<std::mutex> lock(mutex_);
      running_ = false;
    }

    idle_cv_.notify_all();
  }
}

}  // namespace gsplat_rviz_trials
