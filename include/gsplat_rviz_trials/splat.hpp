#ifndef GSPLAT_RVIZ_TRIALS__SPLAT_HPP_
#define GSPLAT_RVIZ_TRIALS__SPLAT_HPP_

#include <memory>
#include <string>
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
#include <OgreGpuProgramParams.h>

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
    const Ogre::ColourValue & color,
    const float sh[48],
    int sh_degree);
  
  virtual ~Splat();

  /**
   * @brief Update the splat orientation to face the camera.
   * @param camera The camera to face.
   */
  void update(Ogre::Camera * camera);

  /**
   * @brief Set the 3D center position of the splat.
   */
  void setCenter(const Ogre::Vector3 & position);

  /**
   * @brief Set the RGBA color of the splat.
   */
  void setColor(const Ogre::ColourValue & color);

  /**
   * @brief Set the 3D covariance matrix components.
   */
  void setCovariance(float v11, float v12, float v13, float v22, float v23, float v33);

  /**
   * @brief Upload spherical harmonics coefficients to the GPU.
   * @param sh     48 floats in coefficient-major order: sh[3*i..3*i+2] = (R,G,B) for basis i.
   * @param sh_degree  Highest SH degree present (0–3).  Unused slots in sh[] must be zero.
   */
  void setSphericalHarmonics(const float sh[48], int sh_degree);

private:
  Ogre::GpuProgramParametersSharedPtr getVertexParams();

  std::unique_ptr<rviz_rendering::MeshShape> mesh_shape_;
  std::string material_name_;   // unique clone owned by this instance
  float covariance_[6];
  Ogre::Vector3 center_;
  Ogre::ColourValue color_;
};

}  // namespace gsplat_rviz_trials

#endif  // GSPLAT_RVIZ_TRIALS__SPLAT_HPP_
