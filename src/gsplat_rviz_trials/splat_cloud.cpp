#include "gsplat_rviz_trials/splat_cloud.hpp"

// GLEW must be included before any other OpenGL header.
#include <GL/glew.h>

#include <algorithm>
#include <numeric>

#include <boost/sort/sort.hpp>

#include <OgreCamera.h>
#include <OgreHardwareBufferManager.h>
#include <OgreMaterialManager.h>
#include <OgreRenderQueue.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
#include <OgreVertexIndexData.h>
#include <OgreViewport.h>

namespace gsplat_rviz_trials
{

static const Ogre::String MOT_SPLAT_CLOUD = "SplatCloud";

SplatCloud::SplatCloud(Ogre::SceneNode * parent_node)
{
  scene_manager_ = parent_node->getCreator();
  buildQuadGeometry();

  material_ = Ogre::MaterialManager::getSingleton().getByName(
    "gsplat_rviz_trials/GaussianSplat", "rviz_rendering");
  material_->load();

  parent_node->attachObject(this);
  scene_manager_->addRenderObjectListener(this);

  sort_thread_ = std::thread(&SplatCloud::sortWorkerMain, this);
}

SplatCloud::~SplatCloud()
{
  // Stop the sort worker before tearing down the data it touches.
  {
    std::lock_guard<std::mutex> lock(sort_mutex_);
    sort_shutdown_        = true;
    sort_request_pending_ = false;
  }
  sort_wake_cv_.notify_all();
  if (sort_thread_.joinable()) {
    sort_thread_.join();
  }

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
  if (tbo_tex_) { glDeleteTextures(1, &tbo_tex_); tbo_tex_ = 0; }
  if (tbo_buf_) { glDeleteBuffers(1, &tbo_buf_);  tbo_buf_ = 0; }
}

void SplatCloud::uploadTBO()
{
  destroyTBO();
  if (pending_splats_.empty()) return;

  const int num_sh      = (active_sh_degree_ + 1) * (active_sh_degree_ + 1);
  texels_per_splat_     = 3 + num_sh;

  // Pack only the active SH coefficients — one vec4 (16 B) per texel.
  std::vector<float> packed(static_cast<size_t>(splat_count_) * texels_per_splat_ * 4);
  for (uint32_t i = 0; i < splat_count_; i++) {
    const SplatGPU & s = pending_splats_[i];
    float * d = packed.data() + static_cast<size_t>(i) * texels_per_splat_ * 4;
    d[0]  = s.center[0]; d[1]  = s.center[1]; d[2]  = s.center[2]; d[3]  = s.alpha;
    d[4]  = s.covA[0];   d[5]  = s.covA[1];   d[6]  = s.covA[2];   d[7]  = 0.0f;
    d[8]  = s.covB[0];   d[9]  = s.covB[1];   d[10] = s.covB[2];   d[11] = 0.0f;
    for (int c = 0; c < num_sh; c++) {
      d[12 + c * 4]     = s.sh[c][0];
      d[12 + c * 4 + 1] = s.sh[c][1];
      d[12 + c * 4 + 2] = s.sh[c][2];
      d[12 + c * 4 + 3] = 0.0f;
    }
  }

  glGenBuffers(1, &tbo_buf_);
  glBindBuffer(GL_TEXTURE_BUFFER, tbo_buf_);
  glBufferData(
    GL_TEXTURE_BUFFER,
    static_cast<GLsizeiptr>(packed.size() * sizeof(float)),
    packed.data(),
    GL_STATIC_DRAW);
  glBindBuffer(GL_TEXTURE_BUFFER, 0);

  glGenTextures(1, &tbo_tex_);
  glBindTexture(GL_TEXTURE_BUFFER, tbo_tex_);
  glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA32F, tbo_buf_);
  glBindTexture(GL_TEXTURE_BUFFER, 0);
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
  // Ensure the worker is not mid-sort before we mutate centers_/sort_indices_/upload_buf_.
  waitForSortIdle();

  splat_count_      = static_cast<uint32_t>(splats.size());
  max_sh_degree_    = sh_degree;
  active_sh_degree_ = std::min(1, sh_degree);
  pending_splats_   = std::move(splats);
  upload_pending_ = true;

  // Rebuild bounds from splat centres
  bounds_.setNull();
  for (const auto & s : pending_splats_) {
    bounds_.merge(Ogre::Vector3(s.center[0], s.center[1], s.center[2]));
  }

  buildIndexVBO();

  // Extract centres into a compact array for cache-friendly depth computation each frame.
  // sort_indices_ is intentionally NOT reset after the first load so that subsequent calls
  // (e.g. reload) start from a fresh identity order; pdqsort exploits the previous frame's
  // order between calls to setSplats(), not across them.
  centers_.resize(splat_count_);
  for (uint32_t i = 0; i < splat_count_; i++) {
    centers_[i] = Ogre::Vector3(
      pending_splats_[i].center[0],
      pending_splats_[i].center[1],
      pending_splats_[i].center[2]);
  }
  depth_keys_.resize(splat_count_);
  sort_indices_.resize(splat_count_);
  std::iota(sort_indices_.begin(), sort_indices_.end(), 0u);
  upload_buf_.resize(splat_count_);

  // Invalidate any stale completed result carried over from the previous load.
  {
    std::lock_guard<std::mutex> lock(sort_mutex_);
    sort_result_ready_ = false;
    upload_buf_front_.clear();
    front_count_       = 0;
  }

  if (auto * node = getParentSceneNode()) {
    node->needUpdate();
  }
}

void SplatCloud::clear()
{
  waitForSortIdle();

  splat_count_      = 0;
  max_sh_degree_    = 0;
  active_sh_degree_ = 0;
  texels_per_splat_ = 0;
  pending_splats_.clear();
  upload_pending_ = false;
  destroyTBO();
  index_vbo_.reset();
  bounds_.setNull();
  centers_.clear();
  depth_keys_.clear();
  sort_indices_.clear();
  upload_buf_.clear();

  {
    std::lock_guard<std::mutex> lock(sort_mutex_);
    sort_result_ready_ = false;
    upload_buf_front_.clear();
    front_count_       = 0;
  }

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

// ── MovableObject ─────────────────────────────────────────────────────────────

const Ogre::String & SplatCloud::getMovableType() const { return MOT_SPLAT_CLOUD; }

Ogre::Real SplatCloud::getBoundingRadius() const
{
  return bounds_.isNull() ? 0.0f : bounds_.getHalfSize().length();
}

void SplatCloud::_updateRenderQueue(Ogre::RenderQueue * queue)
{
  if (splat_count_ == 0) return;

  auto * vp = scene_manager_->getCurrentViewport();
  const Ogre::Camera * cam = vp ? vp->getCamera() : nullptr;

  // Post the current view direction to the worker and pick up any completed
  // sort produced since the last frame. The VBO upload happens under the lock
  // so the worker cannot swap upload_buf_front_ out from under us mid-read —
  // orphaning writeData is typically <1 ms while the sort itself is 100s of ms,
  // so the contention window is negligible.
  if (cam) {
    const Ogre::Vector3 fwd = cam->getDerivedDirection();
    std::lock_guard<std::mutex> lock(sort_mutex_);

    if (sort_result_ready_ && front_count_ == splat_count_ && index_vbo_) {
      index_vbo_->writeData(
        0, front_count_ * sizeof(float),
        upload_buf_front_.data(), true);
      sort_result_ready_ = false;
    }

    sort_req_fwd_         = fwd;
    sort_request_pending_ = true;
  }
  sort_wake_cv_.notify_one();

  queue->addRenderable(this, 95u, mRenderQueuePriority);
}

// ── Sort worker ───────────────────────────────────────────────────────────────

void SplatCloud::waitForSortIdle()
{
  std::unique_lock<std::mutex> lock(sort_mutex_);
  sort_request_pending_ = false;   // cancel any queued request so the worker doesn't pick it up
  sort_idle_cv_.wait(lock, [this] { return !sort_running_; });
}

void SplatCloud::sortWorkerMain()
{
  while (true) {
    Ogre::Vector3 fwd;
    uint32_t count = 0;
    {
      std::unique_lock<std::mutex> lock(sort_mutex_);
      sort_wake_cv_.wait(lock, [this] { return sort_shutdown_ || sort_request_pending_; });
      if (sort_shutdown_) return;

      fwd                   = sort_req_fwd_;
      count                 = splat_count_;
      sort_request_pending_ = false;
      sort_running_         = true;
    }

    // The main thread guarantees (via waitForSortIdle) that these arrays are
    // consistently sized while we hold sort_running_ == true. The size check
    // is belt-and-braces in case of a bug.
    if (count > 0 &&
        centers_.size() == count &&
        depth_keys_.size() == count &&
        sort_indices_.size() == count)
    {
      for (uint32_t i = 0; i < count; ++i) {
        depth_keys_[i] = fwd.dotProduct(centers_[i]);
      }

      // sort_indices_ retains the previous frame's order so pdqsort only fixes
      // the inversions caused by camera movement (near O(n) for small drags).
      boost::sort::pdqsort(
        sort_indices_.begin(), sort_indices_.end(),
        [this](uint32_t a, uint32_t b) {
          return depth_keys_[a] > depth_keys_[b];  // descending = back-to-front
        });

      if (upload_buf_.size() != count) upload_buf_.resize(count);
      for (uint32_t i = 0; i < count; ++i) {
        upload_buf_[i] = static_cast<float>(sort_indices_[i]);
      }

      {
        std::lock_guard<std::mutex> lock(sort_mutex_);
        std::swap(upload_buf_front_, upload_buf_);
        front_count_       = count;
        sort_result_ready_ = true;
        sort_running_      = false;
      }
    } else {
      std::lock_guard<std::mutex> lock(sort_mutex_);
      sort_running_ = false;
    }

    sort_idle_cv_.notify_all();
  }
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
  const Ogre::Pass *,
  const Ogre::AutoParamDataSource *,
  const Ogre::LightList *,
  bool)
{
  if (rend != this) return;

  if (upload_pending_) {
    uploadTBO();
    upload_pending_ = false;
  }

  auto params = material_->getTechnique(0)->getPass(0)->getVertexProgramParameters();
  if (params) {
    params->setNamedConstant("sh_degree", active_sh_degree_);
    params->setNamedConstant("u_stride",  texels_per_splat_);
  }

  if (tbo_tex_) {
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_BUFFER, tbo_tex_);
  }
}

}  // namespace gsplat_rviz_trials
