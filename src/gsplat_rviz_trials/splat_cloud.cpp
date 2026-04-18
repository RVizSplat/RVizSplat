#include "gsplat_rviz_trials/splat_cloud.hpp"

#include <OgreSceneNode.h>
#include <OgreRenderQueue.h>
#include <OgreHardwareBufferManager.h>
#include <OgreVertexIndexData.h>

namespace gsplat_rviz_trials
{

static const Ogre::String MOT_SPLAT_CLOUD = "SplatCloud";

SplatCloud::SplatCloud(Ogre::SceneNode * parent_node)
{
  buildSharedGeometry();
  parent_node->attachObject(this);
}

void SplatCloud::buildSharedGeometry()
{
  // Quad corners at ±2 in x/y; gl_Vertex.xy in the shader reads these as
  // the per-vertex scale factors for the projected 2D Gaussian ellipse.
  static constexpr float kVerts[] = {
    -2.0f, -2.0f, 0.0f,
     2.0f, -2.0f, 0.0f,
     2.0f,  2.0f, 0.0f,
    -2.0f,  2.0f, 0.0f,
  };
  static constexpr uint16_t kIdx[] = {0, 1, 2, 0, 2, 3};

  render_op_.operationType = Ogre::RenderOperation::OT_TRIANGLE_LIST;
  render_op_.useIndexes = true;

  render_op_.vertexData = new Ogre::VertexData();
  render_op_.vertexData->vertexStart = 0;
  render_op_.vertexData->vertexCount = 4;
  render_op_.vertexData->vertexDeclaration->addElement(
    0, 0, Ogre::VET_FLOAT3, Ogre::VES_POSITION);

  auto & bm = Ogre::HardwareBufferManager::getSingleton();
  auto vbuf = bm.createVertexBuffer(
    3 * sizeof(float), 4,
    Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
  vbuf->writeData(0, vbuf->getSizeInBytes(), kVerts, true);
  render_op_.vertexData->vertexBufferBinding->setBinding(0, vbuf);

  auto ibuf = bm.createIndexBuffer(
    Ogre::HardwareIndexBuffer::IT_16BIT, 6,
    Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
  ibuf->writeData(0, ibuf->getSizeInBytes(), kIdx, true);

  render_op_.indexData = new Ogre::IndexData();
  render_op_.indexData->indexBuffer = ibuf;
  render_op_.indexData->indexCount = 6;
  render_op_.indexData->indexStart = 0;
}

SplatCloud::~SplatCloud()
{
  // Detach before tearing down renderables so the scene manager won't
  // try to render them mid-destruction.
  if (auto * node = getParentSceneNode()) {
    node->detachObject(this);
  }
  renderables_.clear();

  // RenderOperation holds raw pointers — free manually.
  delete render_op_.vertexData;
  render_op_.vertexData = nullptr;
  delete render_op_.indexData;
  render_op_.indexData = nullptr;
}

void SplatCloud::addSplat(
  const Ogre::Vector3 & position,
  const float covariance[6],
  const Ogre::ColourValue & color,
  const float sh[48],
  int sh_degree)
{
  renderables_.push_back(
    std::make_unique<Splat>(this, position, covariance, color, sh, sh_degree));
  bounds_.merge(position);
}

void SplatCloud::clear()
{
  renderables_.clear();
  bounds_.setNull();
  if (auto * node = getParentSceneNode()) {
    node->needUpdate();
  }
}

const Ogre::String & SplatCloud::getMovableType() const
{
  return MOT_SPLAT_CLOUD;
}

Ogre::Real SplatCloud::getBoundingRadius() const
{
  if (bounds_.isNull()) {
    return 0.0f;
  }
  return bounds_.getHalfSize().length();
}

void SplatCloud::_updateRenderQueue(Ogre::RenderQueue * queue)
{
  for (auto & r : renderables_) {
    queue->addRenderable(r.get(), 95u, mRenderQueuePriority);
  }
}

void SplatCloud::visitRenderables(Ogre::Renderable::Visitor * visitor, bool /*debugRenderables*/)
{
  for (auto & r : renderables_) {
    visitor->visit(r.get(), 0, false);
  }
}

}  // namespace gsplat_rviz_trials
