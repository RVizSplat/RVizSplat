#include "gsplat_rviz_trials/splat_loaders/ply_file_source.hpp"

#include "gsplat_rviz_trials/splat_loaders/ply_loader.hpp"

namespace gsplat_rviz_trials
{

PlyFileSource::PlyFileSource(const std::string & path)
{
  pending_ = std::make_unique<LoadResult>();
  pending_->splats = loadPly(path, pending_->error, pending_->sh_degree);
}

std::unique_ptr<LoadResult> PlyFileSource::poll()
{
  return std::move(pending_);
}

}  // namespace gsplat_rviz_trials
