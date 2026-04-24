#include "gsplat_rviz_trials/splat_loaders/ply_file_source.hpp"

#include <utility>

#include "gsplat_rviz_trials/splat_loaders/ply_loader.hpp"

namespace gsplat_rviz_trials
{

PlyFileSource::PlyFileSource(const std::string & path) : path_(path) {}

void PlyFileSource::start(Callback cb)
{
  if (!cb) return;
  LoadResult r;
  r.splats = loadPly(path_, r.error, r.sh_degree);
  cb(std::move(r));
}

}  // namespace gsplat_rviz_trials
