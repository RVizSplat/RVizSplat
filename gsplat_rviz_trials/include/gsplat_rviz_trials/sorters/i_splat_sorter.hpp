#ifndef GSPLAT_RVIZ_TRIALS__I_SPLAT_SORTER_HPP_
#define GSPLAT_RVIZ_TRIALS__I_SPLAT_SORTER_HPP_

#include <cstdint>
#include <memory>
#include <vector>

#include <OgreVector3.h>

namespace gsplat_rviz_trials
{

struct SortResult
{
  std::vector<float> indices;  // back-to-front splat indices cast to float
  uint32_t count = 0;
};

class ISplatSorter
{
public:
  virtual ~ISplatSorter() = default;

  // Called once after new splat data is loaded. May upload to device memory.
  virtual void uploadCenters(const std::vector<Ogre::Vector3> & centers) = 0;

  // Post a sort request. Non-blocking for async backends, synchronous for GPU backends.
  virtual void requestSort(const Ogre::Vector3 & cam_fwd) = 0;

  // Returns the latest completed result, or nullptr if the previous result
  // was already consumed or the sort is still running.
  virtual std::unique_ptr<SortResult> pollResult() = 0;

  // Drain any in-flight work and clear internal state.
  virtual void reset() = 0;

  // Short label for logs / status (e.g. "CPU", "CUDA").
  virtual const char * name() const = 0;
};

}  // namespace gsplat_rviz_trials

#endif  // GSPLAT_RVIZ_TRIALS__I_SPLAT_SORTER_HPP_
