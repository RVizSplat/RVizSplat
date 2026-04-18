#include "gsplat_rviz_trials/splat.hpp"
#include "gsplat_rviz_trials/splat_cloud.hpp"

#include <atomic>
#include <string>

#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <OgreGpuProgramParams.h>
#include <OgreSceneNode.h>
#include <OgreMatrix4.h>

namespace gsplat_rviz_trials
{

static std::string makeUniqueMaterialName()
{
  static std::atomic<int> counter{0};
  return "gsplat_rviz_trials/GaussianSplat_instance_" + std::to_string(counter++);
}

Splat::Splat(
  SplatCloud * cloud,
  const Ogre::Vector3 & position,
  const float covariance[6],
  const Ogre::ColourValue & color,
  const float sh[48],
  int sh_degree)
: cloud_(cloud)
{
  Ogre::MaterialPtr base = Ogre::MaterialManager::getSingleton().getByName(
    "gsplat_rviz_trials/GaussianSplat", "rviz_rendering");
  material_ = base->clone(makeUniqueMaterialName());
  material_->load();

  setCenter(position);
  setColor(color);
  setCovariance(
    covariance[0], covariance[1], covariance[2],
    covariance[3], covariance[4], covariance[5]);
  setSphericalHarmonics(sh, sh_degree);
}

Splat::~Splat()
{
  if (material_) {
    Ogre::MaterialManager::getSingleton().remove(material_);
  }
}

void Splat::getRenderOperation(Ogre::RenderOperation & op)
{
  op = cloud_->getRenderOp();
}

void Splat::getWorldTransforms(Ogre::Matrix4 * xform) const
{
  auto * node = cloud_->getParentSceneNode();
  *xform = node ? node->_getFullTransform() : Ogre::Matrix4::IDENTITY;
}

Ogre::Real Splat::getSquaredViewDepth(const Ogre::Camera * cam) const
{
  return (center_ - cam->getDerivedPosition()).squaredLength();
}

const Ogre::LightList & Splat::getLights() const
{
  return cloud_->queryLights();
}

Ogre::GpuProgramParametersSharedPtr Splat::getVertexParams()
{
  if (!material_) {
    return Ogre::GpuProgramParametersSharedPtr();
  }
  return material_->getTechnique(0)->getPass(0)->getVertexProgramParameters();
}

void Splat::setCenter(const Ogre::Vector3 & position)
{
  center_ = position;
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
