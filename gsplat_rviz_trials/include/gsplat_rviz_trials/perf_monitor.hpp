#ifndef GSPLAT_RVIZ_TRIALS__PERF_MONITOR_HPP_
#define GSPLAT_RVIZ_TRIALS__PERF_MONITOR_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <unordered_map>

namespace gsplat_rviz_trials
{
class PerfMonitor
{
public:
  static PerfMonitor & instance();

  // Record the current wall-clock instant as a completed rendered frame.
  void recordFrame();

  // Named CPU timer pair. stopTimer() stores the elapsed duration so it
  // appears in the next logAll() call. Multiple concurrent keys are supported.
  void startTimer(const std::string & key);
  void stopTimer(const std::string & key);

  // Update GPU buffer size stats. Call once after each TBO (re)upload.
  void setBufferStats(
    uint32_t    splat_count,
    std::size_t tbo_bytes,
    std::size_t sh_tbo_bytes);

  // Emit one throttled ROS INFO line containing all accumulated metrics.
  void logAll();

  PerfMonitor(const PerfMonitor &) = delete;
  PerfMonitor & operator=(const PerfMonitor &) = delete;

private:
  PerfMonitor() = default;

  static constexpr std::size_t         kFpsWindow   = 60;
  static constexpr std::chrono::seconds kLogInterval{2};

  mutable std::mutex mutex_;

  // FPS sliding window
  std::deque<std::chrono::steady_clock::time_point> frame_times_;

  // Named timers
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> timer_starts_;
  std::unordered_map<std::string, double>                                 timer_last_ms_;
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> timer_last_updated_;
  // Per-key throttle for the immediate log emitted by stopTimer().
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> timer_last_logged_;

  // Buffer stats
  uint32_t    splat_count_   = 0;
  std::size_t tbo_bytes_     = 0;
  std::size_t sh_tbo_bytes_  = 0;

  std::chrono::steady_clock::time_point last_log_{};
};

}  // namespace gsplat_rviz_trials

#endif  // GSPLAT_RVIZ_TRIALS__PERF_MONITOR_HPP_
