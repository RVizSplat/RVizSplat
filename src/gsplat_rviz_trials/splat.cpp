#include "gsplat_rviz_trials/splat.hpp"

#include <atomic>
#include <string>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreEntity.h>
#include <OgreSubEntity.h>
#include <OgreMaterial.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <OgreVector3.h>
#include <OgreManualObject.h>
#include <OgreGpuProgramParams.h>

#include "rviz_rendering/objects/mesh_shape.hpp"

namespace gsplat_rviz_trials
{

static std::string makeUniqueMaterialName()
{
  static std::atomic<int> counter{0};
  return "gsplat_rviz_trials/GaussianSplat_instance_" + std::to_string(counter++);
}

Splat::Splat(
  Ogre::SceneManager * scene_manager,
  Ogre::SceneNode * parent_node,
  const Ogre::Vector3 & position,
  const float covariance[6],
  const Ogre::ColourValue & color,
  const float sh[48],
  int sh_degree)
{
  mesh_shape_ = std::make_unique<rviz_rendering::MeshShape>(scene_manager, parent_node);

  Ogre::ManualObject * mo = mesh_shape_->getManualObject();
  mesh_shape_->beginTriangles();

  // Define a larger quad (4x4) to ensure the Gaussian falloff has space
  // We use UVs from 0 to 1
  mo->position(-2.0f, -2.0f, 0.0f); mo->textureCoord(0.0f, 0.0f);
  mo->position( 2.0f, -2.0f, 0.0f); mo->textureCoord(1.0f, 0.0f);
  mo->position( 2.0f,  2.0f, 0.0f); mo->textureCoord(1.0f, 1.0f);
  mo->position(-2.0f,  2.0f, 0.0f); mo->textureCoord(0.0f, 1.0f);

  mesh_shape_->addTriangle(0, 1, 2);
  mesh_shape_->addTriangle(0, 2, 3);
  mesh_shape_->endTriangles();

  // Clone the shared material so each Splat instance has its own GPU parameter set.
  // Without cloning, all instances share the same parameter block and the last
  // setNamedConstant() call wins, making every splat look identical.
  material_name_ = makeUniqueMaterialName();
  Ogre::MaterialPtr base = Ogre::MaterialManager::getSingleton().getByName(
    "gsplat_rviz_trials/GaussianSplat", "rviz_rendering");
  base->clone(material_name_);

  if (mesh_shape_->getEntity()) {
    mesh_shape_->getEntity()->setMaterialName(material_name_);
  }

  // Set initial parameters and sync with GPU
  setCenter(position);
  setColor(color);
  setCovariance(covariance[0], covariance[1], covariance[2], covariance[3], covariance[4], covariance[5]);
  setSphericalHarmonics(sh, sh_degree);
}

Splat::~Splat()
{
  // Remove the per-instance material clone to avoid leaking Ogre resources.
  if (!material_name_.empty()) {
    Ogre::MaterialManager::getSingleton().remove(material_name_);
  }
}

// Returns the vertex-program parameter set for this instance's cloned material.
Ogre::GpuProgramParametersSharedPtr Splat::getVertexParams()
{
  if (!mesh_shape_ || !mesh_shape_->getEntity()) {
    return Ogre::GpuProgramParametersSharedPtr();
  }
  Ogre::SubEntity * sub = mesh_shape_->getEntity()->getSubEntity(0);
  return sub->getTechnique()->getPass(0)->getVertexProgramParameters();
}

void Splat::update(Ogre::Camera * camera)
{
  if (camera && mesh_shape_) {
    mesh_shape_->setOrientation(camera->getDerivedOrientation());
  }
}

void Splat::setCenter(const Ogre::Vector3 & position)
{
  center_ = position;
  mesh_shape_->setPosition(position);

  auto params = getVertexParams();
  if (params) {
    params->setNamedConstant("splat_center", center_);
  }
}

void Splat::setColor(const Ogre::ColourValue & color)
{
  color_ = color;
  mesh_shape_->setColor(color.r, color.g, color.b, color.a);

  auto params = getVertexParams();
  if (params) {
    params->setNamedConstant("splat_color", color_);
  }
}

void Splat::setCovariance(float v11, float v12, float v13, float v22, float v23, float v33)
{
  covariance_[0] = v11; covariance_[1] = v12; covariance_[2] = v13;
  covariance_[3] = v22; covariance_[4] = v23; covariance_[5] = v33;

  auto params = getVertexParams();
  if (params) {
    float sigma_arr[9] = {
      v11, v12, v13,
      v12, v22, v23,
      v13, v23, v33
    };
    params->setNamedConstant("covariance3D", sigma_arr, 3, 3);
  }
}

void Splat::setSphericalHarmonics(const float sh[48], int sh_degree)
{
  auto params = getVertexParams();
  if (!params) return;

  // Upload 16 vec3s (48 floats) as coefficient-major RGB triples.
  // Ogre interprets (count=16, multiple_of=3) as 16 groups of 3 floats → vec3[16].
  params->setNamedConstant("spherical_harmonics", sh, 16, 3);
  params->setNamedConstant("sh_degree", sh_degree);
}

}  // namespace gsplat_rviz_trials
