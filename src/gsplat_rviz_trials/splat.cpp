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

Splat::Splat(Ogre::SceneManager * scene_manager, Ogre::SceneNode * parent_node)
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
  mesh_shape_->setColor(1.0f, 1.0f, 1.0f, 1.0f);

  // Set a default identity-like covariance (small sphere)
  setCovariance(0.1f, 0.0f, 0.0f, 0.1f, 0.0f, 0.1f);
}

Splat::~Splat() = default;

void Splat::update(Ogre::Camera * camera)
{
  if (camera && mesh_shape_) {
    mesh_shape_->setOrientation(camera->getDerivedOrientation());
  }
}

void Splat::setCovariance(float v11, float v12, float v13, float v22, float v23, float v33)
{
  if (!mesh_shape_ || !mesh_shape_->getEntity()) return;

  Ogre::Entity * entity = mesh_shape_->getEntity();
  for (unsigned int i = 0; i < entity->getNumSubEntities(); ++i) {
    Ogre::SubEntity * sub = entity->getSubEntity(i);
    Ogre::GpuProgramParametersSharedPtr params = sub->getTechnique()->getPass(0)->getVertexProgramParameters();
    if (params) {
      // Pass covariance as two vec3s or a matrix. 
      // We'll pass it as a custom parameter for simplicity if we can, 
      // but GpuProgramParameters is more reliable for matrices.
      Ogre::Matrix4 sigma(
        v11, v12, v13, 0.0,
        v12, v22, v23, 0.0,
        v13, v23, v33, 0.0,
        0.0, 0.0, 0.0, 0.0
      );
      params->setNamedConstant("covariance3D_padded", sigma);
    }
  }
}

}  // namespace gsplat_rviz_trials
