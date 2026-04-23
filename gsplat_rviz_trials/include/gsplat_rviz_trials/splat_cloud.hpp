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

class ISplatSorter;

// Single MovableObject+Renderable that draws all Gaussian splats in one
// instanced draw call.  Per-splat data lives in a GL_RGBA32F TBO; a
// per-instance VBO holds the sorted splat indices (back-to-front).
//
// Architecture:
//   quad VBO  (source 0, 4 verts, ±2 corners)  ─┐
//   index VBO (source 1, N floats, sorted idx) ─┤─► glDrawElementsInstanced(N)
//   IBO       (6 indices, two triangles)        ─┘
//   TBO       (N × 2 RGBA32UI texels = packed SplatGPU data, 32 B/splat)
class GSPLAT_RVIZ_TRIALS_PUBLIC SplatCloud
  : public Ogre::MovableObject,
    public Ogre::Renderable,
    public Ogre::RenderObjectListener
{
public:
  explicit SplatCloud(Ogre::SceneNode * parent_node);
  ~SplatCloud() override;

  // Inject the depth-sort backend. Caller retains ownership; must outlive
  // SplatCloud or be cleared with setSorter(nullptr) before destruction.
  void setSorter(ISplatSorter * sorter) { sorter_ = sorter; }

  // Upload new splat data.  Safe to call from the main thread; TBO
  // creation is deferred to the first notifyRenderSingleObject() call
  // so the GL context is guaranteed to be current.
  void setSplats(std::vector<SplatGPU> splats, int sh_degree);
  void clear();

  // Change the active SH degree at runtime (clamped to [0, max available]).
  void setShDegree(int d);
  int  getMaxShDegree() const { return max_sh_degree_; }

  uint32_t getSplatCount() const { return splat_count_; }

  // Write sorted indices into the per-instance VBO. Typically called from
  // _updateRenderQueue() with a result from ISplatSorter::pollResult().
  void applySort(const float * indices, uint32_t count);

  // Axis-aligned clip box in the scene_node_'s local frame.  Splats whose
  // centre falls outside [min, max] are culled at the vertex stage.
  void setClipEnabled(bool v) { clip_enabled_ = v; }
  void setClipBox(const Ogre::Vector3 & mn, const Ogre::Vector3 & mx)
  {
    clip_min_ = mn;
    clip_max_ = mx;
  }

  // When true, WBOIT is handling transparency and the per-frame depth sort
  // is skipped (WBOIT is order-independent).
  void setOitEnabled(bool v) { oit_enabled_ = v; }

  // WBOIT weight-function knobs, pushed as fragment uniforms each frame.
  void setWboitWeightScale(float v)    { wboit_weight_scale_    = v; }
  void setWboitWeightExponent(float v) { wboit_weight_exponent_ = v; }
  void setWboitAlphaDiscard(float v)   { wboit_alpha_discard_   = v; }

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

  Ogre::MaterialPtr material_;
  Ogre::RenderOperation render_op_;
  Ogre::AxisAlignedBox bounds_;
  Ogre::SceneManager * scene_manager_ = nullptr;

  std::vector<SplatGPU> pending_splats_;
  bool upload_pending_ = false;
  int max_sh_degree_    = 0;  // highest degree available in the loaded data
  int active_sh_degree_ = 0;  // degree currently sent to the shader (user-controlled)
  int texels_per_splat_ = 0;  // compact base TBO = 2 uvec4 texels/splat (32 B)
  uint32_t splat_count_ = 0;

  // Raw GL handles (GLuint = uint32_t)
  uint32_t tbo_buf_ = 0;
  uint32_t tbo_tex_ = 0;
  uint32_t sh_tbo_buf_ = 0;   // RGBA16F TBO of non-DC SH coefficients (empty when active_sh_degree_ == 0)
  uint32_t sh_tbo_tex_ = 0;
  int      sh_coeffs_per_splat_ = 0;   // (active_sh_degree_+1)² - 1

  Ogre::HardwareVertexBufferSharedPtr index_vbo_;  // per-instance sorted indices

  ISplatSorter * sorter_ = nullptr;  // not owned

  // ROI clip state (pushed as uniforms each frame).
  bool          clip_enabled_ = false;
  Ogre::Vector3 clip_min_{-1e9f, -1e9f, -1e9f};
  Ogre::Vector3 clip_max_{ 1e9f,  1e9f,  1e9f};

  // Transparency state.
  bool  oit_enabled_           = false;
  float wboit_weight_scale_    = 5.0f;
  float wboit_weight_exponent_ = 2.0f;
  float wboit_alpha_discard_   = 0.01f;
};

}  // namespace gsplat_rviz_trials

#endif  // GSPLAT_RVIZ_TRIALS__SPLAT_CLOUD_HPP_
