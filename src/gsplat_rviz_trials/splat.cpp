#include "gsplat_rviz_trials/splat.hpp"

#include <atomic>
#include <string>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreMaterial.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <OgreHardwareBufferManager.h>
#include <OgreVertexIndexData.h>
#include <OgreGpuProgramParams.h>

namespace gsplat_rviz_trials
{

static std::string makeUniqueMaterialName()
{
  static std::atomic<int> counter{0};
  return "gsplat_rviz_trials/GaussianSplat_instance_" + std::to_string(counter++);
}

Splat::Splat(
  Ogre::SceneManager * /*scene_manager*/,
  Ogre::SceneNode * parent_node,
  const Ogre::Vector3 & position,
  const float covariance[6],
  const Ogre::ColourValue & color,
  const float sh[48],
  int sh_degree)
{
  buildGeometry();

  // Clone the shared base material so each instance has independent GPU param storage.
  Ogre::MaterialPtr base = Ogre::MaterialManager::getSingleton().getByName(
    "gsplat_rviz_trials/GaussianSplat", "rviz_rendering");
  setMaterial(base->clone(makeUniqueMaterialName()));

  // Child node carries position and camera-facing orientation for this splat.
  node_ = parent_node->createChildSceneNode();
  node_->attachObject(this);

  // Transparent objects must be in RENDER_QUEUE_TRANSPARENTS (95) so Ogre
  // depth-sorts them back-to-front using getSquaredViewDepth before blending.
  setRenderQueueGroup(95u);  // RENDER_QUEUE_TRANSPARENTS

  // Local-space bounding box: quad spans ±2 in x/y, nearly flat in z.
  mBox.setExtents(
    Ogre::Vector3(-2.0f, -2.0f, -0.01f),
    Ogre::Vector3( 2.0f,  2.0f,  0.01f));

  setCenter(position);
  setColor(color);
  setCovariance(
    covariance[0], covariance[1], covariance[2],
    covariance[3], covariance[4], covariance[5]);
  setSphericalHarmonics(sh, sh_degree);
}

void Splat::buildGeometry()
{
  // Billboard quad: four corners at ±2 in x/y.
  // gl_Vertex.xy in the vertex shader reads these as vPosition for Gaussian falloff.
  static constexpr float kVerts[] = {
    -2.0f, -2.0f, 0.0f,
     2.0f, -2.0f, 0.0f,
     2.0f,  2.0f, 0.0f,
    -2.0f,  2.0f, 0.0f,
  };
  static constexpr uint16_t kIdx[] = {0, 1, 2, 0, 2, 3};

  mRenderOp.operationType = Ogre::RenderOperation::OT_TRIANGLE_LIST;
  mRenderOp.useIndexes = true;

  // --- vertex data ---
  mRenderOp.vertexData = new Ogre::VertexData();
  mRenderOp.vertexData->vertexStart = 0;
  mRenderOp.vertexData->vertexCount = 4;

  mRenderOp.vertexData->vertexDeclaration->addElement(
    0, 0, Ogre::VET_FLOAT3, Ogre::VES_POSITION);

  auto & bm = Ogre::HardwareBufferManager::getSingleton();
  auto vbuf = bm.createVertexBuffer(
    3 * sizeof(float), 4,
    Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
  vbuf->writeData(0, vbuf->getSizeInBytes(), kVerts, true);
  mRenderOp.vertexData->vertexBufferBinding->setBinding(0, vbuf);

  // --- index data ---
  auto ibuf = bm.createIndexBuffer(
    Ogre::HardwareIndexBuffer::IT_16BIT, 6,
    Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
  ibuf->writeData(0, ibuf->getSizeInBytes(), kIdx, true);

  mRenderOp.indexData = new Ogre::IndexData();
  mRenderOp.indexData->indexBuffer = ibuf;
  mRenderOp.indexData->indexCount = 6;
  mRenderOp.indexData->indexStart = 0;
}

Splat::~Splat()
{
  if (node_) {
    node_->detachAllObjects();
    node_->getCreator()->destroySceneNode(node_);
    node_ = nullptr;
  }
  // Use the ResourcePtr overload so the lookup is by handle — no group name needed.
  if (auto mat = getMaterial()) {
    Ogre::MaterialManager::getSingleton().remove(mat);
  }
  // RenderOperation holds raw pointers — must free manually (base class does not).
  delete mRenderOp.vertexData;
  mRenderOp.vertexData = nullptr;
  delete mRenderOp.indexData;
  mRenderOp.indexData = nullptr;
}

Ogre::Real Splat::getBoundingRadius() const
{
  return 2.0f * Ogre::Math::Sqrt(2.0f);  // diagonal of the ±2 quad
}

Ogre::Real Splat::getSquaredViewDepth(const Ogre::Camera * cam) const
{
  auto diff = mParentNode->_getDerivedPosition() - cam->getDerivedPosition();
  return diff.squaredLength();
}

Ogre::GpuProgramParametersSharedPtr Splat::getVertexParams()
{
  auto mat = getMaterial();
  if (!mat) {
    return Ogre::GpuProgramParametersSharedPtr();
  }
  return mat->getTechnique(0)->getPass(0)->getVertexProgramParameters();
}

void Splat::update(Ogre::Camera * camera)
{
  if (camera && node_) {
    node_->setOrientation(camera->getDerivedOrientation());
  }
}

void Splat::setCenter(const Ogre::Vector3 & position)
{
  center_ = position;
  if (node_) {
    node_->setPosition(position);
  }
  auto params = getVertexParams();
  if (params) {
    params->setNamedConstant("splat_center", center_);
  }
}

void Splat::setColor(const Ogre::ColourValue & color)
{
  color_ = color;
  auto params = getVertexParams();
  if (params) {
    params->setNamedConstant("splat_opacity", color_.a);
  }
}

void Splat::setCovariance(float v11, float v12, float v13, float v22, float v23, float v33)
{
  covariance_[0] = v11; covariance_[1] = v12; covariance_[2] = v13;
  covariance_[3] = v22; covariance_[4] = v23; covariance_[5] = v33;

  auto params = getVertexParams();
  if (params) {
    params->setNamedConstant("cov_row0", Ogre::Vector3(v11, v12, v13));
    params->setNamedConstant("cov_row1", Ogre::Vector3(v22, v23, v33));
  }
}

void Splat::setSphericalHarmonics(const float sh[48], int sh_degree)
{
  auto params = getVertexParams();
  if (!params) {
    return;
  }
  params->setNamedConstant("spherical_harmonics", sh, 16, 3);
  params->setNamedConstant("sh_degree", sh_degree);
}

}  // namespace gsplat_rviz_trials
