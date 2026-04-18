#ifndef GSPLAT_RVIZ_TRIALS__SPLAT_HPP_
#define GSPLAT_RVIZ_TRIALS__SPLAT_HPP_

#include <OgreRenderable.h>
#include <OgreVector3.h>
#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreCamera.h>
#include <OgreGpuProgramParams.h>

#include "gsplat_rviz_trials/visibility_control.hpp"

namespace gsplat_rviz_trials
{

class SplatCloud;

/**
 * @brief One Gaussian splat — a pure Ogre::Renderable backed by a cloned material.
 *
 * Geometry (the shared quad VBO/IBO) is owned by the parent SplatCloud and
 * returned by reference from getRenderOperation().  This class only owns the
 * per-instance material (and therefore the per-instance GPU uniform block).
 */
class GSPLAT_RVIZ_TRIALS_PUBLIC Splat : public Ogre::Renderable
{
public:
  Splat(
    SplatCloud * cloud,
    const Ogre::Vector3 & position,
    const float covariance[6],
    const Ogre::ColourValue & color,
    const float sh[48],
    int sh_degree);

  ~Splat() override;

  void setCenter(const Ogre::Vector3 & position);
  void setColor(const Ogre::ColourValue & color);
  void setCovariance(float v11, float v12, float v13, float v22, float v23, float v33);

  /**
   * @param sh       48 floats, coefficient-major RGB: sh[3*i..3*i+2] = (R,G,B) for basis i.
   * @param sh_degree  Highest SH degree present (0–3).
   */
  void setSphericalHarmonics(const float sh[48], int sh_degree);

  // --- Ogre::Renderable interface ---
  const Ogre::MaterialPtr & getMaterial() const override { return material_; }
  void getRenderOperation(Ogre::RenderOperation & op) override;
  void getWorldTransforms(Ogre::Matrix4 * xform) const override;
  Ogre::Real getSquaredViewDepth(const Ogre::Camera * cam) const override;
  const Ogre::LightList & getLights() const override;

private:
  Ogre::GpuProgramParametersSharedPtr getVertexParams();

  SplatCloud * cloud_;
  Ogre::MaterialPtr material_;
  Ogre::Vector3 center_;
  Ogre::ColourValue color_;
  float covariance_[6]{};
};

}  // namespace gsplat_rviz_trials

#endif  // GSPLAT_RVIZ_TRIALS__SPLAT_HPP_
