#ifndef GSPLAT_RVIZ_TRIALS__SPLAT_CLOUD_HPP_
#define GSPLAT_RVIZ_TRIALS__SPLAT_CLOUD_HPP_

#include <memory>
#include <vector>

#include <OgreMovableObject.h>
#include <OgreAxisAlignedBox.h>
#include <OgreRenderOperation.h>
#include <OgreVector3.h>
#include <OgreColourValue.h>

#include "gsplat_rviz_trials/splat.hpp"
#include "gsplat_rviz_trials/visibility_control.hpp"

namespace Ogre
{
class SceneNode;
class RenderQueue;
}

namespace gsplat_rviz_trials
{

/**
 * @brief Single MovableObject that owns all Gaussian splat Renderables.
 *
 * Attaches to the display's scene node.  Owns one shared quad VBO/IBO that
 * every Splat Renderable returns from getRenderOperation() — only the
 * per-instance materials differ.
 */
class GSPLAT_RVIZ_TRIALS_PUBLIC SplatCloud : public Ogre::MovableObject
{
public:
  explicit SplatCloud(Ogre::SceneNode * parent_node);
  ~SplatCloud() override;

  void addSplat(
    const Ogre::Vector3 & position,
    const float covariance[6],
    const Ogre::ColourValue & color,
    const float sh[48],
    int sh_degree);

  void clear();

  /** Shared render operation returned by every child Splat. */
  const Ogre::RenderOperation & getRenderOp() const { return render_op_; }

  // --- Ogre::MovableObject interface ---
  const Ogre::String & getMovableType() const override;
  const Ogre::AxisAlignedBox & getBoundingBox() const override { return bounds_; }
  Ogre::Real getBoundingRadius() const override;
  void _updateRenderQueue(Ogre::RenderQueue * queue) override;
  void visitRenderables(Ogre::Renderable::Visitor * visitor, bool debugRenderables = false) override;

private:
  void buildSharedGeometry();

  std::vector<std::unique_ptr<Splat>> renderables_;
  Ogre::RenderOperation render_op_;
  Ogre::AxisAlignedBox bounds_;
};

}  // namespace gsplat_rviz_trials

#endif  // GSPLAT_RVIZ_TRIALS__SPLAT_CLOUD_HPP_
