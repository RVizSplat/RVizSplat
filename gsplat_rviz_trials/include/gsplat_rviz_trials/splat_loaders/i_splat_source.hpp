#ifndef GSPLAT_RVIZ_TRIALS__I_SPLAT_SOURCE_HPP_
#define GSPLAT_RVIZ_TRIALS__I_SPLAT_SOURCE_HPP_

#include <functional>
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
  using Callback = std::function<void(LoadResult)>;

  virtual ~ISplatSource() = default;

  virtual void start(Callback cb) = 0;
};

}  // namespace gsplat_rviz_trials

#endif  // GSPLAT_RVIZ_TRIALS__I_SPLAT_SOURCE_HPP_
