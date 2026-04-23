#ifndef GSPLAT_RVIZ_TRIALS__PROFILER_HPP_
#define GSPLAT_RVIZ_TRIALS__PROFILER_HPP_

#include <chrono>
#include <mutex>
#include <string>
#include <unordered_map>

namespace gsplat_rviz_trials
{

// Thread-safe singleton for timing named events.
//
// Usage:
//   Profiler::instance().start("my_event");
//   // ... work ...
//   Profiler::instance().stop("my_event");   // prints elapsed time
class Profiler
{
public:
  static Profiler & instance();

  // Begin timing the event named `key`. If a prior start with the same key
  // has no matching stop, it is overwritten.
  void start(const std::string & key);

  // End timing for `key`: compute elapsed since the matching start, print
  // the result, and drop the entry. No-op if no start was recorded.
  void stop(const std::string & key);

  Profiler(const Profiler &) = delete;
  Profiler & operator=(const Profiler &) = delete;

private:
  Profiler() = default;

  std::mutex mutex_;
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> starts_;
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_logged_;
};

}  // namespace gsplat_rviz_trials

#endif  // GSPLAT_RVIZ_TRIALS__PROFILER_HPP_
