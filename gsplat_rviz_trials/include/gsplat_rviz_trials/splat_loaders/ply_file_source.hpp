#ifndef GSPLAT_RVIZ_TRIALS__PLY_FILE_SOURCE_HPP_
#define GSPLAT_RVIZ_TRIALS__PLY_FILE_SOURCE_HPP_

#include <memory>
#include <string>

#include "gsplat_rviz_trials/splat_loaders/i_splat_source.hpp"

namespace gsplat_rviz_trials
{

// Synchronous PLY loader. load() parses the file on the calling thread and
// stashes the result; poll() returns it exactly once.
class PlyFileSource : public ISplatSource
{
public:
  explicit PlyFileSource(const std::string & path);
  ~PlyFileSource() override = default;

  std::unique_ptr<LoadResult> poll() override;

private:
  std::unique_ptr<LoadResult> pending_;
};

}  // namespace gsplat_rviz_trials

#endif  // GSPLAT_RVIZ_TRIALS__PLY_FILE_SOURCE_HPP_
