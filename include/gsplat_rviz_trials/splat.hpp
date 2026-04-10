#ifndef GSPLAT_RVIZ_TRIALS__SPLAT_HPP_
#define GSPLAT_RVIZ_TRIALS__SPLAT_HPP_

#include <memory>

#include "gsplat_rviz_trials/visibility_control.hpp"

namespace Ogre
{
class SceneManager;
class SceneNode;
}

namespace rviz_rendering
{
class MeshShape;
}

namespace gsplat_rviz_trials
{

/**
 * @brief A simple white rectangle object built using MeshShape.
 */
class GSPLAT_RVIZ_TRIALS_PUBLIC Splat
{
public:
  /**
   * @brief Constructor
   * @param scene_manager The Ogre scene manager
   * @param parent_node The parent scene node to attach to
   */
  Splat(Ogre::SceneManager * scene_manager, Ogre::SceneNode * parent_node);
  virtual ~Splat();

private:
  std::unique_ptr<rviz_rendering::MeshShape> mesh_shape_;
};

}  // namespace gsplat_rviz_trials

#endif  // GSPLAT_RVIZ_TRIALS__SPLAT_HPP_
