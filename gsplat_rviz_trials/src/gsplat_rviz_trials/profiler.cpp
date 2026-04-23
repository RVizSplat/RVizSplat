#include "gsplat_rviz_trials/profiler.hpp"

#include <rclcpp/logging.hpp>

namespace gsplat_rviz_trials
{
namespace
{
// Minimum interval between two prints of the same key. First occurrence of a
// key is always logged; subsequent calls within this window are suppressed.
constexpr std::chrono::seconds kThrottleInterval{2};
}  // namespace

Profiler & Profiler::instance()
{
  static Profiler p;
  return p;
}

void Profiler::start(const std::string & key)
{
  const auto t = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex_);
  starts_[key] = t;
}

void Profiler::stop(const std::string & key)
{
  const auto end = std::chrono::steady_clock::now();

  std::chrono::steady_clock::time_point begin;
  bool should_log = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = starts_.find(key);
    if (it == starts_.end()) return;
    begin = it->second;
    starts_.erase(it);

    // Per-key throttle: default-constructed time_point (epoch) is far enough
    // in the past that the first stop() always passes.
    auto & last = last_logged_[key];
    if (end - last >= kThrottleInterval) {
      last = end;
      should_log = true;
    }
  }

  if (!should_log) return;

  const double ms = std::chrono::duration<double, std::milli>(end - begin).count();
  RCLCPP_INFO(
    rclcpp::get_logger("gsplat_rviz_trials"),
    "[profiler] %s: %.3f ms", key.c_str(), ms);
}

}  // namespace gsplat_rviz_trials
