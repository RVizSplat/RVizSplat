#ifndef GSPLAT_RVIZ_TRIALS__SPLAT_CLOUD_HPP_
#define GSPLAT_RVIZ_TRIALS__SPLAT_CLOUD_HPP_

#include <cstdint>
#include <vector>

#include <OgreAxisAlignedBox.h>
#include <OgreMaterial.h>
#include <OgreMovableObject.h>
#include <OgreRenderable.h>
#include <OgreRenderObjectListener.h>
#include <OgreRenderOperation.h>
#include <OgreVector3.h>

#include "gsplat_rviz_trials/splat_gpu.hpp"
#include "gsplat_rviz_trials/visibility_control.hpp"

namespace Ogre
{
class RenderQueue;
class SceneManager;
class SceneNode;
}

namespace gsplat_rviz_trials
{

// Single MovableObject+Renderable that draws all Gaussian splats in one
// instanced draw call.  Per-splat data lives in a GL_RGBA32F TBO; a
// per-instance VBO holds the sorted splat indices (back-to-front).
//
// Architecture:
//   quad VBO  (source 0, 4 verts, ±2 corners)  ─┐
//   index VBO (source 1, N floats, sorted idx) ─┤─► glDrawElementsInstanced(N)
//   IBO       (6 indices, two triangles)        ─┘
//   TBO       (N × 19 RGBA32F texels = SplatGPU data)
class GSPLAT_RVIZ_TRIALS_PUBLIC SplatCloud
  : public Ogre::MovableObject,
    public Ogre::Renderable,
    public Ogre::RenderObjectListener
{
public:
  explicit SplatCloud(Ogre::SceneNode * parent_node);
  ~SplatCloud() override;

  // Upload new splat data.  Safe to call from the main thread; TBO
  // creation is deferred to the first notifyRenderSingleObject() call
  // so the GL context is guaranteed to be current.
  void setSplats(std::vector<SplatGPU> splats, int sh_degree);
  void clear();

  // ── Ogre::MovableObject ────────────────────────────────────────────
  const Ogre::String & getMovableType() const override;
  const Ogre::AxisAlignedBox & getBoundingBox() const override { return bounds_; }
  Ogre::Real getBoundingRadius() const override;
  void _updateRenderQueue(Ogre::RenderQueue * queue) override;
  void visitRenderables(Ogre::Renderable::Visitor * visitor, bool debugRenderables = false) override;

  // ── Ogre::Renderable ──────────────────────────────────────────────
  const Ogre::MaterialPtr & getMaterial() const override { return material_; }
  void getRenderOperation(Ogre::RenderOperation & op) override;
  void getWorldTransforms(Ogre::Matrix4 * xform) const override;
  Ogre::Real getSquaredViewDepth(const Ogre::Camera * cam) const override;
  const Ogre::LightList & getLights() const override;

  // ── Ogre::RenderObjectListener ────────────────────────────────────
  // Binds the TBO to texture unit 0 immediately before the draw call.
  void notifyRenderSingleObject(
    Ogre::Renderable * rend,
    const Ogre::Pass * pass,
    const Ogre::AutoParamDataSource * source,
    const Ogre::LightList * pLightList,
    bool suppressRenderStateChanges) override;

private:
  void buildQuadGeometry();
  void destroyTBO();
  void uploadTBO();       // must be called with GL context current
  void buildIndexVBO();   // allocates the per-instance index VBO
  void sortSplats();      // depth-sorts and uploads index VBO every frame

  Ogre::MaterialPtr material_;
  Ogre::RenderOperation render_op_;
  Ogre::AxisAlignedBox bounds_;
  Ogre::SceneManager * scene_manager_ = nullptr;

  std::vector<SplatGPU> pending_splats_;
  bool upload_pending_ = false;
  int sh_degree_ = 0;
  uint32_t splat_count_ = 0;

  // Raw GL handles (GLuint = uint32_t)
  uint32_t tbo_buf_ = 0;
  uint32_t tbo_tex_ = 0;

  Ogre::HardwareVertexBufferSharedPtr index_vbo_;  // per-instance sorted indices

  // Sorting state — persists between frames so pdqsort sees near-sorted input
  std::vector<Ogre::Vector3> centers_;      // splat centres (cache-friendly, separate from TBO data)
  std::vector<float>         depth_keys_;   // depth per original splat index
  std::vector<uint32_t>      sort_indices_; // sorted permutation, kept from last frame
  std::vector<float>         upload_buf_;   // float cast of sort_indices_ for VBO upload
};

}  // namespace gsplat_rviz_trials

#endif  // GSPLAT_RVIZ_TRIALS__SPLAT_CLOUD_HPP_
