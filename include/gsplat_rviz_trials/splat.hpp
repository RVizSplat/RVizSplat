#ifndef GSPLAT_RVIZ_TRIALS__SPLAT_HPP_
#define GSPLAT_RVIZ_TRIALS__SPLAT_HPP_

#include <memory>
#include <OgreCamera.h>

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

#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <OgreColourValue.h>

namespace gsplat_rviz_trials
{

/**
 * @brief A Gaussian Splat object rendered using a billboarding quad.
 */
class GSPLAT_RVIZ_TRIALS_PUBLIC Splat
{
public:
  /**
   * @brief Constructor
   * @param scene_manager The Ogre scene manager
   * @param parent_node The parent scene node to attach to
   * @param position 3D position of the splat
   * @param covariance symmetric components: v11, v12, v13, v22, v23, v33
   * @param color RGBA color/opacity
   */
  Splat(
    Ogre::SceneManager * scene_manager, 
    Ogre::SceneNode * parent_node,
    const Ogre::Vector3 & position,
    const float covariance[6],
    const Ogre::ColourValue & color);
  
  virtual ~Splat();

  /**
   * @brief Update the splat orientation to face the camera.
   * @param camera The camera to face.
   */
  void update(Ogre::Camera * camera);

  /**
   * @brief Set the 3D covariance matrix components.
   * Matrix is symmetric: [v11, v12, v13]
   *                      [v12, v22, v23]
   *                      [v13, v23, v33]
   */
  void setCovariance(float v11, float v12, float v13, float v22, float v23, float v33);

private:
  std::unique_ptr<rviz_rendering::MeshShape> mesh_shape_;
  float covariance_[6];
};

}  // namespace gsplat_rviz_trials

#endif  // GSPLAT_RVIZ_TRIALS__SPLAT_HPP_
