#include "gsplat_rviz_trials/splat_cloud.hpp"

// GLEW must be included before any other OpenGL header.
#include <GL/glew.h>

#ifdef GSPLAT_USE_CUDA
#include <cuda_gl_interop.h>
#include <cuda_runtime.h>
#include "gsplat_rviz_trials/cuda_gl_bridge.hpp"
#endif

#include <algorithm>
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

#ifdef GSPLAT_USE_CUDA
  if (cuda_vbo_resource_) {
    cudaGraphicsUnregisterResource(
      reinterpret_cast<cudaGraphicsResource_t>(cuda_vbo_resource_));
    cuda_vbo_resource_ = nullptr;
  }
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

#ifdef GSPLAT_USE_CUDA
  if (use_cuda_) {
    if (cuda_vbo_resource_) {
      cudaGraphicsUnregisterResource(
        reinterpret_cast<cudaGraphicsResource_t>(cuda_vbo_resource_));
      cuda_vbo_resource_ = nullptr;
      cuda_sorter_.destroy();
    }
    cuda_registration_pending_ = true;
  }
#endif
}

// ── Public API ────────────────────────────────────────────────────────────────

void SplatCloud::setSplats(std::vector<SplatGPU> splats, int sh_degree)
{
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
  centers_.clear();
  depth_keys_.clear();
  sort_indices_.clear();
  upload_buf_.clear();

#ifdef GSPLAT_USE_CUDA
  if (cuda_vbo_resource_) {
    cudaGraphicsUnregisterResource(
      reinterpret_cast<cudaGraphicsResource_t>(cuda_vbo_resource_));
    cuda_vbo_resource_ = nullptr;
  }
  cuda_sorter_.destroy();
  cuda_registration_pending_ = false;
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
  sortSplats();
  queue->addRenderable(this, 95u, mRenderQueuePriority);
}

// ── Sorting ───────────────────────────────────────────────────────────────────

void SplatCloud::sortSplats()
{
  auto * vp = scene_manager_->getCurrentViewport();
  if (!vp) return;
  const Ogre::Camera * cam = vp->getCamera();
  if (!cam) return;

#ifdef GSPLAT_USE_CUDA
  if (use_cuda_ && cuda_vbo_resource_) {
    auto res = reinterpret_cast<cudaGraphicsResource_t>(cuda_vbo_resource_);
    void * mapped_ptr = nullptr;
    size_t nbytes = 0;
    bool gpu_sort_ok = false;

    cudaError_t err = cudaGraphicsMapResources(1, &res, /*stream=*/0);
    if (err != cudaSuccess) {
      RCLCPP_ERROR_ONCE(
        rclcpp::get_logger("gsplat_rviz_trials"),
        "SplatCloud: cudaGraphicsMapResources failed (%s) — falling back to pdqsort",
        cudaGetErrorString(err));
    } else {
      err = cudaGraphicsResourceGetMappedPointer(&mapped_ptr, &nbytes, res);
      if (err != cudaSuccess || !mapped_ptr) {
        RCLCPP_ERROR_ONCE(
          rclcpp::get_logger("gsplat_rviz_trials"),
          "SplatCloud: cudaGraphicsResourceGetMappedPointer failed (%s)",
          cudaGetErrorString(err));
      } else {
        const Ogre::Vector3 fwd = cam->getDerivedDirection();
        const float cam_fwd[3] = {fwd.x, fwd.y, fwd.z};
        gpu_sort_ok = cuda_sorter_.sort(mapped_ptr, cam_fwd, splat_count_);
        if (!gpu_sort_ok) {
          RCLCPP_ERROR_ONCE(
            rclcpp::get_logger("gsplat_rviz_trials"),
            "SplatCloud: CUDA sort failed — falling back to pdqsort");
        }
      }
      cudaGraphicsUnmapResources(1, &res, /*stream=*/0);
    }

    if (gpu_sort_ok) return;
    // fall through to pdqsort
  }
#endif

  const Ogre::Vector3 fwd = cam->getDerivedDirection();

  // Sequential depth computation — centres_ is a compact array so this is cache-friendly.
  for (uint32_t i = 0; i < splat_count_; i++) {
    depth_keys_[i] = fwd.dotProduct(centers_[i]);
  }

  // sort_indices_ retains the previous frame's order so pdqsort only fixes
  // the inversions caused by the camera movement (near O(n) for small drags).
  boost::sort::pdqsort(
    sort_indices_.begin(), sort_indices_.end(),
    [this](uint32_t a, uint32_t b) {
      return depth_keys_[a] > depth_keys_[b];  // descending = back-to-front
    });

  // Cast to float for the per-instance VBO attribute (float is exact for N < 2^24).
  for (uint32_t i = 0; i < splat_count_; i++) {
    upload_buf_[i] = static_cast<float>(sort_indices_[i]);
  }

  index_vbo_->writeData(0, splat_count_ * sizeof(float), upload_buf_.data(), false);
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

#ifdef GSPLAT_USE_CUDA
  if (cuda_registration_pending_ && use_cuda_ && index_vbo_) {
    cuda_registration_pending_ = false;

    // Verify that the current GL context has an associated CUDA device.
    // This fails on Optimus/hybrid-GPU laptops where GL runs on the iGPU
    // but CUDA runs on the dGPU — the two cannot share buffer objects.
    unsigned int gl_device_count = 0;
    int gl_device = -1;
    cudaError_t gl_err = cudaGLGetDevices(
      &gl_device_count, &gl_device, 1, cudaGLDeviceListAll);
    if (gl_err != cudaSuccess || gl_device_count == 0) {
      RCLCPP_ERROR(
        rclcpp::get_logger("gsplat_rviz_trials"),
        "SplatCloud: no CUDA device for current GL context (%s) — "
        "GL is likely on the integrated GPU (Optimus); disabling GPU sort",
        gl_err == cudaSuccess ? "no devices found" : cudaGetErrorString(gl_err));
      use_cuda_ = false;
    } else {
      RCLCPP_INFO(
        rclcpp::get_logger("gsplat_rviz_trials"),
        "SplatCloud: CUDA-GL interop available on device %d", gl_device);

      const uint32_t gl_buf_id = getGLBufferId(index_vbo_.get());
      cudaGraphicsResource_t res;
      cudaError_t err = cudaGraphicsGLRegisterBuffer(
        &res, gl_buf_id, cudaGraphicsMapFlagsNone);
      if (err != cudaSuccess) {
        RCLCPP_ERROR(
          rclcpp::get_logger("gsplat_rviz_trials"),
          "SplatCloud: cudaGraphicsGLRegisterBuffer(buf=%u) failed (%s) — disabling GPU sort",
          gl_buf_id, cudaGetErrorString(err));
        use_cuda_ = false;
      } else {
        cuda_vbo_resource_ = res;
        cuda_sorter_.init(splat_count_);
        cuda_sorter_.uploadCenters(&centers_[0].x, splat_count_);
        RCLCPP_INFO(
          rclcpp::get_logger("gsplat_rviz_trials"),
          "SplatCloud: CUDA-GL interop registered for %u splats (GL buf %u)",
          splat_count_, gl_buf_id);
      }
    }
  }
#endif

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
