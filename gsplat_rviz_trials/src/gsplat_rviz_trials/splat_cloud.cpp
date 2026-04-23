#include "gsplat_rviz_trials/splat_cloud.hpp"

// GLEW must be included before any other OpenGL header.
#include <GL/glew.h>

#include <algorithm>
#include <cstring>
#include <limits>
#include <numeric>

#include <OgreAutoParamDataSource.h>
#include <OgreCamera.h>
#include <OgreHardwareBufferManager.h>
#include <OgreMaterialManager.h>
#include <OgreRenderQueue.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
#include <OgreVertexIndexData.h>
#include <OgreViewport.h>

#include "gsplat_rviz_trials/sorters/i_splat_sorter.hpp"

namespace gsplat_rviz_trials
{

static const Ogre::String MOT_SPLAT_CLOUD = "SplatCloud";

// IEEE 754 float32 → binary16. Round-to-nearest-even, saturates to inf on overflow.
static inline uint16_t floatToHalf(float f)
{
  uint32_t x;
  std::memcpy(&x, &f, 4);
  const uint32_t sign = (x >> 16) & 0x8000u;
  const uint32_t e32  = (x >> 23) & 0xFFu;
  const uint32_t m32  = x & 0x7FFFFFu;

  if (e32 == 0xFFu) {                          // Inf / NaN
    return uint16_t(sign | 0x7C00u | (m32 ? 0x200u : 0u));
  }
  int e = int(e32) - 127 + 15;
  if (e >= 0x1F) return uint16_t(sign | 0x7C00u);  // overflow → inf
  if (e <= 0) {                                // subnormal / zero / underflow
    if (e < -10) return uint16_t(sign);
    const uint32_t m = m32 | 0x800000u;
    const uint32_t shift = uint32_t(14 - e);
    const uint32_t half  = (m >> shift) + ((m >> (shift - 1)) & 1u);
    return uint16_t(sign | half);
  }
  uint32_t m = (m32 >> 13) + ((m32 >> 12) & 1u);
  if (m & 0x400u) { m = 0; ++e; if (e >= 0x1F) return uint16_t(sign | 0x7C00u); }
  return uint16_t(sign | (uint32_t(e) << 10) | m);
}

static inline uint32_t packHalf2x16(float a, float b)
{
  return uint32_t(floatToHalf(a)) | (uint32_t(floatToHalf(b)) << 16);
}

// DC SH coefficient (Y_0^0). RGB = SH_C0 * sh[0] + 0.5, clamp [0,1].
static constexpr float kShC0 = 0.28209479177387814f;

static inline uint32_t packDCColor(const float sh0[3], float alpha)
{
  auto quant = [](float v) {
    v = v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
    return uint32_t(v * 255.0f + 0.5f) & 0xFFu;
  };
  const uint32_t r = quant(kShC0 * sh0[0] + 0.5f);
  const uint32_t g = quant(kShC0 * sh0[1] + 0.5f);
  const uint32_t b = quant(kShC0 * sh0[2] + 0.5f);
  const uint32_t a = quant(alpha);
  return r | (g << 8) | (b << 16) | (a << 24);
}

SplatCloud::SplatCloud(Ogre::SceneNode * parent_node)
{
  scene_manager_ = parent_node->getCreator();

  buildQuadGeometry();

  material_ = Ogre::MaterialManager::getSingleton().getByName(
    "gsplat_rviz_trials/GaussianSplat", "rviz_rendering");
  material_->load();

  parent_node->attachObject(this);
  scene_manager_->addRenderObjectListener(this);
}

SplatCloud::~SplatCloud()
{
  if (scene_manager_) {
    scene_manager_->removeRenderObjectListener(this);
  }
  if (auto * node = getParentSceneNode()) {
    node->detachObject(this);
  }
  destroyTBO();

  delete render_op_.vertexData;
  delete render_op_.indexData;
}

// ── Geometry ──────────────────────────────────────────────────────────────────

void SplatCloud::buildQuadGeometry()
{
  // Four quad corners; gl_Vertex.xy in the shader reads these as the ±scale
  // factors for the projected 2D Gaussian eigenvectors.
  static constexpr float kVerts[] = {
    -2.0f, -2.0f,
     2.0f, -2.0f,
     2.0f,  2.0f,
    -2.0f,  2.0f,
  };
  static constexpr uint16_t kIdx[] = {0, 1, 2, 0, 2, 3};

  render_op_.operationType = Ogre::RenderOperation::OT_TRIANGLE_LIST;
  render_op_.useIndexes    = true;

  render_op_.vertexData = new Ogre::VertexData();
  render_op_.vertexData->vertexStart = 0;
  render_op_.vertexData->vertexCount = 4;

  // Source 0: per-vertex quad position (VES_POSITION → Ogre GL attribute "vertex", location 0)
  // Source 1: per-instance sorted index (VES_TEXTURE_COORDINATES[0] → "uv0", location 8)
  render_op_.vertexData->vertexDeclaration->addElement(
    0, 0, Ogre::VET_FLOAT2, Ogre::VES_POSITION);
  render_op_.vertexData->vertexDeclaration->addElement(
    1, 0, Ogre::VET_FLOAT1, Ogre::VES_TEXTURE_COORDINATES, 0);

  auto & bm = Ogre::HardwareBufferManager::getSingleton();

  auto quad_vbo = bm.createVertexBuffer(
    2 * sizeof(float), 4, Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
  quad_vbo->writeData(0, quad_vbo->getSizeInBytes(), kVerts, true);
  render_op_.vertexData->vertexBufferBinding->setBinding(0, quad_vbo);

  auto ibuf = bm.createIndexBuffer(
    Ogre::HardwareIndexBuffer::IT_16BIT, 6,
    Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
  ibuf->writeData(0, ibuf->getSizeInBytes(), kIdx, true);

  render_op_.indexData = new Ogre::IndexData();
  render_op_.indexData->indexBuffer = ibuf;
  render_op_.indexData->indexCount  = 6;
  render_op_.indexData->indexStart  = 0;
}

// ── TBO lifecycle ─────────────────────────────────────────────────────────────

void SplatCloud::destroyTBO()
{
  if (tbo_tex_)    { glDeleteTextures(1, &tbo_tex_);    tbo_tex_ = 0; }
  if (tbo_buf_)    { glDeleteBuffers(1, &tbo_buf_);     tbo_buf_ = 0; }
  if (sh_tbo_tex_) { glDeleteTextures(1, &sh_tbo_tex_); sh_tbo_tex_ = 0; }
  if (sh_tbo_buf_) { glDeleteBuffers(1, &sh_tbo_buf_);  sh_tbo_buf_ = 0; }
  sh_coeffs_per_splat_ = 0;
}

void SplatCloud::uploadTBO()
{
  destroyTBO();
  if (pending_splats_.empty()) return;

  // Compact base TBO: 2 uvec4 texels per splat (32 B total).
  //   texel 0: center.xyz as float-bits, packHalf2x16(cov00, cov01)
  //   texel 1: packHalf2x16(cov02, cov11), packHalf2x16(cov12, cov22),
  //            packUnorm4x8(DC-baked RGBA), reserved
  texels_per_splat_ = 2;
  std::vector<uint32_t> packed(static_cast<size_t>(splat_count_) * 8);
  for (uint32_t i = 0; i < splat_count_; ++i) {
    const SplatGPU & s = pending_splats_[i];
    uint32_t * d = packed.data() + static_cast<size_t>(i) * 8;

    std::memcpy(&d[0], &s.center[0], 4);
    std::memcpy(&d[1], &s.center[1], 4);
    std::memcpy(&d[2], &s.center[2], 4);
    // Covariance upper-triangle: covA = {v11,v12,v13}, covB = {v22,v23,v33}
    d[3] = packHalf2x16(s.covA[0], s.covA[1]);  // v11, v12
    d[4] = packHalf2x16(s.covA[2], s.covB[0]);  // v13, v22
    d[5] = packHalf2x16(s.covB[1], s.covB[2]);  // v23, v33
    d[6] = packDCColor(s.sh[0], s.alpha);
    d[7] = 0u;
  }

  glGenBuffers(1, &tbo_buf_);
  glBindBuffer(GL_TEXTURE_BUFFER, tbo_buf_);
  glBufferData(
    GL_TEXTURE_BUFFER,
    static_cast<GLsizeiptr>(packed.size() * sizeof(uint32_t)),
    packed.data(),
    GL_STATIC_DRAW);
  glBindBuffer(GL_TEXTURE_BUFFER, 0);

  glGenTextures(1, &tbo_tex_);
  glBindTexture(GL_TEXTURE_BUFFER, tbo_tex_);
  glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA32UI, tbo_buf_);
  glBindTexture(GL_TEXTURE_BUFFER, 0);

  // ── Optional SH TBO for non-DC coefficients (GL_RGBA16F, 8 B/coeff) ──
  sh_coeffs_per_splat_ =
    (active_sh_degree_ + 1) * (active_sh_degree_ + 1) - 1;  // excludes DC
  if (sh_coeffs_per_splat_ > 0) {
    // Each non-DC coefficient packs into one RGBA16F texel: (R, G, B, 0).
    // Layout is [splat0_c1, splat0_c2, …, splat1_c1, splat1_c2, …].
    std::vector<uint16_t> sh_packed(
      static_cast<size_t>(splat_count_) * sh_coeffs_per_splat_ * 4);
    for (uint32_t i = 0; i < splat_count_; ++i) {
      const SplatGPU & s = pending_splats_[i];
      uint16_t * d = sh_packed.data() +
        static_cast<size_t>(i) * sh_coeffs_per_splat_ * 4;
      for (int c = 0; c < sh_coeffs_per_splat_; ++c) {
        // SplatGPU::sh[0] is DC; non-DC starts at sh[1].
        d[c * 4 + 0] = floatToHalf(s.sh[c + 1][0]);
        d[c * 4 + 1] = floatToHalf(s.sh[c + 1][1]);
        d[c * 4 + 2] = floatToHalf(s.sh[c + 1][2]);
        d[c * 4 + 3] = 0;
      }
    }

    glGenBuffers(1, &sh_tbo_buf_);
    glBindBuffer(GL_TEXTURE_BUFFER, sh_tbo_buf_);
    glBufferData(
      GL_TEXTURE_BUFFER,
      static_cast<GLsizeiptr>(sh_packed.size() * sizeof(uint16_t)),
      sh_packed.data(),
      GL_STATIC_DRAW);
    glBindBuffer(GL_TEXTURE_BUFFER, 0);

    glGenTextures(1, &sh_tbo_tex_);
    glBindTexture(GL_TEXTURE_BUFFER, sh_tbo_tex_);
    glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA16F, sh_tbo_buf_);
    glBindTexture(GL_TEXTURE_BUFFER, 0);
  }
}

// ── Per-instance index VBO ────────────────────────────────────────────────────

void SplatCloud::buildIndexVBO()
{
  if (splat_count_ == 0) { index_vbo_.reset(); return; }

  auto & bm = Ogre::HardwareBufferManager::getSingleton();
  index_vbo_ = bm.createVertexBuffer(
    sizeof(float), splat_count_,
    Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

  // Identity sort: instance i draws splat i
  std::vector<float> ids(splat_count_);
  std::iota(ids.begin(), ids.end(), 0.0f);
  index_vbo_->writeData(0, index_vbo_->getSizeInBytes(), ids.data(), true);

  index_vbo_->setIsInstanceData(true);
  index_vbo_->setInstanceDataStepRate(1);

  render_op_.vertexData->vertexBufferBinding->setBinding(1, index_vbo_);
}

// ── Public API ────────────────────────────────────────────────────────────────

void SplatCloud::setSplats(std::vector<SplatGPU> splats, int sh_degree)
{
  splat_count_      = static_cast<uint32_t>(splats.size());
  max_sh_degree_    = sh_degree;
  active_sh_degree_ = std::min(1, sh_degree);  // default: SH1 if available, else 0
  pending_splats_   = std::move(splats);
  upload_pending_   = true;

  // Rebuild bounds from splat centres
  bounds_.setNull();
  for (const auto & s : pending_splats_) {
    bounds_.merge(Ogre::Vector3(s.center[0], s.center[1], s.center[2]));
  }

  buildIndexVBO();

  if (auto * node = getParentSceneNode()) {
    node->needUpdate();
  }
}

void SplatCloud::clear()
{
  splat_count_      = 0;
  max_sh_degree_    = 0;
  active_sh_degree_ = 0;
  texels_per_splat_ = 0;
  pending_splats_.clear();
  upload_pending_ = false;
  destroyTBO();
  index_vbo_.reset();
  bounds_.setNull();

  if (auto * node = getParentSceneNode()) {
    node->needUpdate();
  }
}

void SplatCloud::setShDegree(int d)
{
  const int clamped = std::clamp(d, 0, max_sh_degree_);
  if (clamped == active_sh_degree_) return;
  active_sh_degree_ = clamped;
  upload_pending_   = true;  // triggers re-pack + re-upload in notifyRenderSingleObject
}

void SplatCloud::applySort(const float * indices, uint32_t count)
{
  if (!index_vbo_ || count == 0 || count != splat_count_) return;
  index_vbo_->writeData(0, count * sizeof(float), indices, true);
}

// ── MovableObject ─────────────────────────────────────────────────────────────

const Ogre::String & SplatCloud::getMovableType() const { return MOT_SPLAT_CLOUD; }

Ogre::Real SplatCloud::getBoundingRadius() const
{
  return bounds_.isNull() ? 0.0f : bounds_.getHalfSize().length();
}

void SplatCloud::_updateRenderQueue(Ogre::RenderQueue * queue)
{
  if (splat_count_ == 0) return;

  // WBOIT is order-independent; skip the sort when it's active.
  if (sorter_ && !oit_enabled_) {
    auto * vp = scene_manager_->getCurrentViewport();
    if (const Ogre::Camera * cam = vp ? vp->getCamera() : nullptr) {
      sorter_->requestSort(cam->getDerivedDirection());
      if (auto result = sorter_->pollResult()) {
        applySort(result->indices.data(), result->count);
      }
    }
  }

  // Move the splat out of the opaque range when the WBOIT compositor is
  // active so its pass-1 opaque scene (lastRenderQueue=94) skips the splats
  // and passes 2-3 (first/last=95) can pick them up with the right scheme.
  const uint8_t queue_group = oit_enabled_ ? 95u : 50u;
  queue->addRenderable(this, queue_group, mRenderQueuePriority);
}

void SplatCloud::visitRenderables(Ogre::Renderable::Visitor * visitor, bool /*debug*/)
{
  if (splat_count_ > 0) {
    visitor->visit(this, 0, false);
  }
}

// ── Renderable ────────────────────────────────────────────────────────────────

void SplatCloud::getRenderOperation(Ogre::RenderOperation & op)
{
  op = render_op_;
  op.numberOfInstances = splat_count_;
}

void SplatCloud::getWorldTransforms(Ogre::Matrix4 * xform) const
{
  auto * node = getParentSceneNode();
  *xform = node ? node->_getFullTransform() : Ogre::Matrix4::IDENTITY;
}

Ogre::Real SplatCloud::getSquaredViewDepth(const Ogre::Camera * cam) const
{
  if (bounds_.isNull()) return 0.0f;
  return (bounds_.getCenter() - cam->getDerivedPosition()).squaredLength();
}

const Ogre::LightList & SplatCloud::getLights() const { return queryLights(); }

// ── RenderObjectListener ──────────────────────────────────────────────────────

void SplatCloud::notifyRenderSingleObject(
  Ogre::Renderable * rend,
  const Ogre::Pass * pass,
  const Ogre::AutoParamDataSource * source,
  const Ogre::LightList *,
  bool)
{
  if (rend != this) return;

  if (upload_pending_) {
    uploadTBO();
    upload_pending_ = false;
  }

  // Splat-tight forward-z range in view space — WBOIT's weight function needs
  // a meaningful [0,1] depth range instead of the ~[0.98, 0.9998] cluster you
  // get from raw gl_FragCoord.z.  Sentinel (far = -1) → shader falls back to
  // gl_FragCoord.z.
  float splat_z_near = 0.0f;
  float splat_z_far  = -1.0f;
  if (oit_enabled_ && source && !bounds_.isNull()) {
    const Ogre::Matrix4 & view = source->getViewMatrix();
    const auto corners = bounds_.getAllCorners();
    float z_min =  std::numeric_limits<float>::max();
    float z_max = -std::numeric_limits<float>::max();
    for (int i = 0; i < 8; ++i) {
      const Ogre::Vector3 vc = view * corners[i];
      const float fwd = -vc.z;   // Ogre view space: -Z forward
      if (fwd < z_min) z_min = fwd;
      if (fwd > z_max) z_max = fwd;
    }
    if (z_max > z_min + 1e-3f) {
      const float span = z_max - z_min;
      splat_z_near = std::max(0.0f, z_min - 0.05f * span);
      splat_z_far  = z_max + 0.05f * span;
    }
  }

  // Push per-frame uniforms on the pass being rendered.  setIgnoreMissingParams
  // lets us fire the whole bundle regardless of which shader declares what.
  if (pass->hasVertexProgram()) {
    auto params = pass->getVertexProgramParameters();
    if (params) {
      params->setIgnoreMissingParams(true);
      params->setNamedConstant("sh_degree",      active_sh_degree_);
      params->setNamedConstant("u_clip_enabled", clip_enabled_ ? 1 : 0);
      const Ogre::Vector4 cmin(clip_min_.x, clip_min_.y, clip_min_.z, 0.0f);
      const Ogre::Vector4 cmax(clip_max_.x, clip_max_.y, clip_max_.z, 0.0f);
      params->setNamedConstant("u_clip_min",     cmin);
      params->setNamedConstant("u_clip_max",     cmax);
      params->setNamedConstant("u_splat_z_near", splat_z_near);
      params->setNamedConstant("u_splat_z_far",  splat_z_far);
    }
  }
  if (pass->hasFragmentProgram()) {
    auto fp = pass->getFragmentProgramParameters();
    if (fp) {
      fp->setIgnoreMissingParams(true);
      fp->setNamedConstant("wboit_weight_scale",    wboit_weight_scale_);
      fp->setNamedConstant("wboit_weight_exponent", wboit_weight_exponent_);
      fp->setNamedConstant("wboit_alpha_discard",   wboit_alpha_discard_);
    }
  }

  if (tbo_tex_) {
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_BUFFER, tbo_tex_);
  }

  // Bind SH TBO on unit 1 when it exists; otherwise leave whatever is there —
  // the shader only samples it when sh_degree > 0.
  glActiveTexture(GL_TEXTURE1);
  glBindTexture(GL_TEXTURE_BUFFER, sh_tbo_tex_);
  glActiveTexture(GL_TEXTURE0);
}

}  // namespace gsplat_rviz_trials
