#include "gsplat_rviz_trials/splat_cloud.hpp"

// GLEW must be included before any other OpenGL header.
#include <GL/glew.h>

#ifdef GSPLAT_USE_CUDA
#include <cuda_runtime.h>
#endif

#include <algorithm>
#include <chrono>
#include <cstring>
#include <numeric>

#include <rclcpp/logging.hpp>

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

#ifdef GSPLAT_USE_CUDA
  {
    int n = 0;
    if (cudaGetDeviceCount(&n) == cudaSuccess && n > 0) {
      use_cuda_ = true;
    }
  }
  RCLCPP_INFO(
    rclcpp::get_logger("gsplat_rviz_trials"),
    "SplatCloud: splat sort = %s", use_cuda_ ? "CUDA GPU (CUB radix sort)" : "CPU (pdqsort)");
#endif

  last_stats_log_ = std::chrono::steady_clock::now();
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

#ifdef GSPLAT_USE_CUDA
  cuda_sorter_.destroy();
#endif

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

#ifdef GSPLAT_USE_CUDA
  if (use_cuda_) {
    cuda_sorter_.destroy();
    cuda_sorter_.init(splat_count_);
  }
#endif
}

// ── Public API ────────────────────────────────────────────────────────────────

void SplatCloud::setSplats(std::vector<SplatGPU> splats, int sh_degree)
{
  // Ensure the worker is not mid-sort before we mutate centers_/sort_indices_/upload_buf_.
  waitForSortIdle();

  splat_count_      = static_cast<uint32_t>(splats.size());
  max_sh_degree_    = sh_degree;
  active_sh_degree_ = 0;  // lean-mode Phase 1: DC only (baked into base TBO)
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

#ifdef GSPLAT_USE_CUDA
  if (use_cuda_ && splat_count_ > 0) {
    cuda_sorter_.uploadCenters(&centers_[0].x, splat_count_);
  }
#endif

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

#ifdef GSPLAT_USE_CUDA
  cuda_sorter_.destroy();
#endif

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
#ifdef GSPLAT_USE_CUDA
    // CUDA fast path: CUB radix sort in ~1-2 ms for ~1M splats, so we run it
    // inline on the render thread rather than deferring to a worker.
    if (use_cuda_ && cuda_sorter_.h_vals_out && index_vbo_) {
      const auto t0 = std::chrono::high_resolution_clock::now();
      const Ogre::Vector3 fwd = cam->getDerivedDirection();
      const float cam_fwd[3] = {fwd.x, fwd.y, fwd.z};
      if (cuda_sorter_.sort(cam_fwd, splat_count_)) {
        index_vbo_->writeData(
          0, splat_count_ * sizeof(float), cuda_sorter_.h_vals_out, false);
        const double ms = std::chrono::duration<double, std::milli>(
          std::chrono::high_resolution_clock::now() - t0).count();
        recordSortTime(ms, "CUDA");
        queue->addRenderable(this, 95u, mRenderQueuePriority);
        return;
      }
      RCLCPP_ERROR_ONCE(
        rclcpp::get_logger("gsplat_rviz_trials"),
        "SplatCloud: CUDA sort failed — falling back to CPU worker sort");
    }
#endif

    // CPU fallback: post the view direction to the worker, pick up any result
    // completed since the last frame.
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
      const auto t0 = std::chrono::high_resolution_clock::now();

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

      const double ms = std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - t0).count();
      recordSortTime(ms, "pdqsort");

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

void SplatCloud::recordSortTime(double ms, const char * method)
{
  stat_sum_ -= sort_times_ms_[stat_head_];
  sort_times_ms_[stat_head_] = ms;
  stat_sum_ += ms;
  stat_head_ = (stat_head_ + 1) % kStatWindow;
  if (stat_count_ < kStatWindow) ++stat_count_;

  const auto now = std::chrono::steady_clock::now();
  if (std::chrono::duration<double>(now - last_stats_log_).count() >= 2.0) {
    last_stats_log_ = now;
    const double avg = stat_sum_ / stat_count_;
    double mn = sort_times_ms_[0];
    double mx = sort_times_ms_[0];
    for (int i = 1; i < stat_count_; i++) {
      if (sort_times_ms_[i] < mn) mn = sort_times_ms_[i];
      if (sort_times_ms_[i] > mx) mx = sort_times_ms_[i];
    }
    RCLCPP_INFO(
      rclcpp::get_logger("gsplat_rviz_trials"),
      "Sort [%-7s] %u splats | cur %.2fms  avg %.2fms  min %.2fms  max %.2fms  (n=%d)",
      method, splat_count_, ms, avg, mn, mx, stat_count_);
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

  if (tbo_tex_) {
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_BUFFER, tbo_tex_);
  }
}

}  // namespace gsplat_rviz_trials
