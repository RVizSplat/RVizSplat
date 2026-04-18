#ifndef GSPLAT_RVIZ_TRIALS__SPLAT_HPP_
#define GSPLAT_RVIZ_TRIALS__SPLAT_HPP_

#include <OgreSimpleRenderable.h>
#include <OgreVector3.h>
#include <OgreColourValue.h>
#include <OgreGpuProgramParams.h>
#include <OgreCamera.h>

#include "gsplat_rviz_trials/visibility_control.hpp"

namespace Ogre
{
class SceneManager;
class SceneNode;
}

namespace gsplat_rviz_trials
{

/**
 * @brief A Gaussian Splat billboard backed by a raw Ogre VBO/IBO via SimpleRenderable.
 *
 * Each instance owns a cloned material (for independent per-GPU uniforms) and a
 * child SceneNode used for position and camera-facing orientation.
 */
class GSPLAT_RVIZ_TRIALS_PUBLIC Splat : public Ogre::SimpleRenderable
{
public:
  Splat(
    Ogre::SceneManager * scene_manager,
    Ogre::SceneNode * parent_node,
    const Ogre::Vector3 & position,
    const float covariance[6],
    const Ogre::ColourValue & color,
    const float sh[48],
    int sh_degree);

  ~Splat() override;

  /** Rotate the billboard to face the camera each frame. */
  void update(Ogre::Camera * camera);

  void setCenter(const Ogre::Vector3 & position);
  void setColor(const Ogre::ColourValue & color);
  void setCovariance(float v11, float v12, float v13, float v22, float v23, float v33);

  /**
   * @param sh     48 floats, coefficient-major RGB: sh[3*i..3*i+2] = (R,G,B) for basis i.
   * @param sh_degree  Highest SH degree present (0–3).
   */
  void setSphericalHarmonics(const float sh[48], int sh_degree);

  // --- Ogre::SimpleRenderable interface ---
  Ogre::Real getBoundingRadius() const override;
  Ogre::Real getSquaredViewDepth(const Ogre::Camera * cam) const override;

private:
  void buildGeometry();
  Ogre::GpuProgramParametersSharedPtr getVertexParams();

  Ogre::SceneNode * node_{nullptr};
  float covariance_[6]{};
  Ogre::Vector3 center_;
  Ogre::ColourValue color_;
};

}  // namespace gsplat_rviz_trials

#endif  // GSPLAT_RVIZ_TRIALS__SPLAT_HPP_
