#include "gsplat_rviz_plugin/perf_monitor.hpp"

#include <rclcpp/logging.hpp>

namespace gsplat_rviz_plugin
{

PerfMonitor & PerfMonitor::instance()
{
  static PerfMonitor p;
  return p;
}

void PerfMonitor::recordFrame()
{
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex_);
  frame_times_.push_back(now);
  if (frame_times_.size() > kFpsWindow) {
    frame_times_.pop_front();
  }
}

void PerfMonitor::startTimer(const std::string & key)
{
  const auto t = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex_);
  timer_starts_[key] = t;
}

void PerfMonitor::stopTimer(const std::string & key)
{
  const auto end = std::chrono::steady_clock::now();

  double ms         = 0.0;
  bool   should_log = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = timer_starts_.find(key);
    if (it == timer_starts_.end()) return;
    ms = std::chrono::duration<double, std::milli>(end - it->second).count();
    timer_last_ms_[key]      = ms;
    timer_last_updated_[key] = end;
    timer_starts_.erase(it);

    auto & last = timer_last_logged_[key];
    if (end - last >= kLogInterval) {
      last       = end;
      should_log = true;
    }
  }

  if (should_log) {
    RCLCPP_INFO(
      rclcpp::get_logger("gsplat_rviz_plugin"),
      "[perf] %s: %.3f ms", key.c_str(), ms);
  }
}

void PerfMonitor::setBufferStats(
  uint32_t    splat_count,
  std::size_t tbo_bytes,
  std::size_t sh_tbo_bytes)
{
  std::lock_guard<std::mutex> lock(mutex_);
  splat_count_  = splat_count;
  tbo_bytes_    = tbo_bytes;
  sh_tbo_bytes_ = sh_tbo_bytes;
}

void PerfMonitor::logAll()
{
  const auto now = std::chrono::steady_clock::now();

  float       fps         = 0.0f;
  double      sort_ms     = 0.0;
  double      render_ms   = 0.0;
  bool        sort_cuda   = false;
  uint32_t    splat_count;
  std::size_t tbo_bytes, sh_tbo_bytes;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (now - last_log_ < kLogInterval) return;
    last_log_ = now;

    // Sliding-window FPS: (frames - 1) events span the window duration.
    if (frame_times_.size() >= 2) {
      const double dt =
        std::chrono::duration<double>(
          frame_times_.back() - frame_times_.front()).count();
      if (dt > 0.0) {
        fps = static_cast<float>((frame_times_.size() - 1) / dt);
      }
    }

    auto last_ms = [&](const std::string & key) -> double {
      auto it = timer_last_ms_.find(key);
      return (it != timer_last_ms_.end()) ? it->second : 0.0;
    };
    auto last_updated = [&](const std::string & key) -> std::chrono::steady_clock::time_point {
      auto it = timer_last_updated_.find(key);
      return (it != timer_last_updated_.end()) ? it->second
                                               : std::chrono::steady_clock::time_point{};
    };

    // Always use whichever sort backend was written most recently to avoid
    // reporting a stale value from a previously active backend.
    const bool cuda_newer =
      last_updated("cuda_sort") >= last_updated("cpu_sort") &&
      last_updated("cuda_sort") != std::chrono::steady_clock::time_point{};
    sort_cuda = cuda_newer;
    sort_ms   = cuda_newer ? last_ms("cuda_sort") : last_ms("cpu_sort");
    render_ms = last_ms("render");

    splat_count  = splat_count_;
    tbo_bytes    = tbo_bytes_;
    sh_tbo_bytes = sh_tbo_bytes_;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("gsplat_rviz_plugin"),
    "[perf] FPS: %.1f  splats: %u"
    "  |  sort (%s): %.3f ms  render: %.3f ms"
    "  |  TBO: %.2f MB  SH-TBO: %.2f MB",
    fps, splat_count,
    sort_cuda ? "cuda" : "cpu", sort_ms, render_ms,
    static_cast<double>(tbo_bytes)    / (1024.0 * 1024.0),
    static_cast<double>(sh_tbo_bytes) / (1024.0 * 1024.0));
}

}  // namespace gsplat_rviz_plugin
