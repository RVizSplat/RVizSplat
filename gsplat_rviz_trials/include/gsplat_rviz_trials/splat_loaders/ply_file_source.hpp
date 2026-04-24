#ifndef GSPLAT_RVIZ_TRIALS__PLY_FILE_SOURCE_HPP_
#define GSPLAT_RVIZ_TRIALS__PLY_FILE_SOURCE_HPP_

#include <string>

#include "gsplat_rviz_trials/splat_loaders/i_splat_source.hpp"

namespace gsplat_rviz_trials
{

// Synchronous PLY loader. start() parses the file on the calling thread and
// invokes the callback exactly once before returning.
class PlyFileSource : public ISplatSource
{
public:
  explicit PlyFileSource(const std::string & path);
  ~PlyFileSource() override = default;

  void start(Callback cb) override;

private:
  std::string path_;
};

}  // namespace gsplat_rviz_trials

#endif  // GSPLAT_RVIZ_TRIALS__PLY_FILE_SOURCE_HPP_
