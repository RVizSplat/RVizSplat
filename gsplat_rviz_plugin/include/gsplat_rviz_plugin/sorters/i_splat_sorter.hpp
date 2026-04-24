#ifndef GSPLAT_RVIZ_PLUGIN__I_SPLAT_SORTER_HPP_
#define GSPLAT_RVIZ_PLUGIN__I_SPLAT_SORTER_HPP_

#include <cstdint>
#include <vector>

#include <OgreVector3.h>

namespace gsplat_rviz_plugin
{

struct SortResult
{
  std::vector<float> indices;  // back-to-front splat indices cast to float
  uint32_t count = 0;
};

// Synchronous depth-sort backend. Implementations only sort — they do not
// own threads or schedule work. Callers decide what thread these methods
// run on (see SplatCloud's internal SortScheduler).
class ISplatSorter
{
public:
  virtual ~ISplatSorter() = default;

  // Upload splat centres. Pass an empty vector to release all state.
  virtual void uploadCenters(const std::vector<Ogre::Vector3> & centers) = 0;

  // Blocking sort. Returns back-to-front indices. Empty result on error
  // or when no centres have been uploaded.
  virtual SortResult sort(const Ogre::Vector3 & cam_fwd) = 0;

  // Short label for logs / status (e.g. "CPU", "CUDA").
  virtual const char * name() const = 0;
};

}  // namespace gsplat_rviz_plugin

#endif  // GSPLAT_RVIZ_PLUGIN__I_SPLAT_SORTER_HPP_
