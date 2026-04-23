#ifndef GSPLAT_RVIZ_TRIALS__I_SPLAT_SOURCE_HPP_
#define GSPLAT_RVIZ_TRIALS__I_SPLAT_SOURCE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "gsplat_rviz_trials/splat_gpu.hpp"

namespace gsplat_rviz_trials
{

struct LoadResult
{
  std::vector<SplatGPU> splats;
  int sh_degree = 0;
  std::string error;  // non-empty on failure

  bool ok() const { return error.empty(); }
};

class ISplatSource
{
public:
  virtual ~ISplatSource() = default;

  // Poll for new data. Returns a LoadResult (success or error) when one is
  // ready, otherwise nullptr. Must be called from the main thread.
  virtual std::unique_ptr<LoadResult> poll() = 0;
};

}  // namespace gsplat_rviz_trials

#endif  // GSPLAT_RVIZ_TRIALS__I_SPLAT_SOURCE_HPP_
