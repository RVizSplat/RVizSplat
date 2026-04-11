#include "gsplat_rviz_trials/splat.hpp"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreEntity.h>
#include <OgreSubEntity.h>
#include <OgreMaterial.h>
#include <OgreTechnique.h>
#include <OgreVector3.h>
#include <OgreManualObject.h>
#include <OgreGpuProgramParams.h>

#include "rviz_rendering/objects/mesh_shape.hpp"

namespace gsplat_rviz_trials
{

Splat::Splat(
  Ogre::SceneManager * scene_manager, 
  Ogre::SceneNode * parent_node,
  const Ogre::Vector3 & position,
  const float covariance[6],
  const Ogre::ColourValue & color)
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
  
  if (mesh_shape_->getEntity()) {
    mesh_shape_->getEntity()->setMaterialName("gsplat_rviz_trials/GaussianSplat", "rviz_rendering");
  }

  // Set initial parameters and sync with GPU
  setCenter(position);
  setColor(color);
  setCovariance(covariance[0], covariance[1], covariance[2], covariance[3], covariance[4], covariance[5]);
}

Splat::~Splat() = default;

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

  if (!mesh_shape_ || !mesh_shape_->getEntity()) return;

  Ogre::Entity * entity = mesh_shape_->getEntity();
  for (unsigned int i = 0; i < entity->getNumSubEntities(); ++i) {
    Ogre::SubEntity * sub = entity->getSubEntity(i);
    Ogre::GpuProgramParametersSharedPtr params = sub->getTechnique()->getPass(0)->getVertexProgramParameters();
    if (params) {
      params->setNamedConstant("splat_center", center_);
    }
  }
}

void Splat::setColor(const Ogre::ColourValue & color)
{
  color_ = color;
  mesh_shape_->setColor(color.r, color.g, color.b, color.a);

  if (!mesh_shape_ || !mesh_shape_->getEntity()) return;

  Ogre::Entity * entity = mesh_shape_->getEntity();
  for (unsigned int i = 0; i < entity->getNumSubEntities(); ++i) {
    Ogre::SubEntity * sub = entity->getSubEntity(i);
    Ogre::GpuProgramParametersSharedPtr params = sub->getTechnique()->getPass(0)->getVertexProgramParameters();
    if (params) {
      params->setNamedConstant("splat_color", color_);
    }
  }
}

void Splat::setCovariance(float v11, float v12, float v13, float v22, float v23, float v33)
{
  covariance_[0] = v11; covariance_[1] = v12; covariance_[2] = v13;
  covariance_[3] = v22; covariance_[4] = v23; covariance_[5] = v33;

  if (!mesh_shape_ || !mesh_shape_->getEntity()) return;

  Ogre::Entity * entity = mesh_shape_->getEntity();
  for (unsigned int i = 0; i < entity->getNumSubEntities(); ++i) {
    Ogre::SubEntity * sub = entity->getSubEntity(i);
    Ogre::GpuProgramParametersSharedPtr params = sub->getTechnique()->getPass(0)->getVertexProgramParameters();
    if (params) {
      float sigma_arr[9] = {
        v11, v12, v13,
        v12, v22, v23,
        v13, v23, v33
      };
      params->setNamedConstant("covariance3D", sigma_arr, 3, 3);
    }
  }
}

}  // namespace gsplat_rviz_trials
