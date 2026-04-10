#include "gsplat_rviz_trials/splat.hpp"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreEntity.h>
#include <OgreMaterial.h>
#include <OgreTechnique.h>
#include <OgreVector3.h>
#include <OgreManualObject.h>

#include "rviz_rendering/objects/mesh_shape.hpp"

namespace gsplat_rviz_trials
{

Splat::Splat(Ogre::SceneManager * scene_manager, Ogre::SceneNode * parent_node)
{
  mesh_shape_ = std::make_unique<rviz_rendering::MeshShape>(scene_manager, parent_node);

  // We need to provide UV coordinates for the Gaussian shader.
  // MeshShape's addVertex wrappers don't support UVs, so we use the ManualObject directly.
  Ogre::ManualObject * mo = mesh_shape_->getManualObject();
  
  // Call beginTriangles to ensure MeshShape's internal state is correct.
  mesh_shape_->beginTriangles();
  
  // Define a 2x1 rectangle centered at origin with UVs
  // Vertex 0
  mo->position(-1.0f, -0.5f, 0.0f);
  mo->textureCoord(0.0f, 0.0f);
  // Vertex 1
  mo->position( 1.0f, -0.5f, 0.0f);
  mo->textureCoord(1.0f, 0.0f);
  // Vertex 2
  mo->position( 1.0f,  0.5f, 0.0f);
  mo->textureCoord(1.0f, 1.0f);
  // Vertex 3
  mo->position(-1.0f,  0.5f, 0.0f);
  mo->textureCoord(0.0f, 1.0f);
  
  // Add triangles using indices
  mesh_shape_->addTriangle(0, 1, 2);
  mesh_shape_->addTriangle(0, 2, 3);
  
  mesh_shape_->endTriangles();
  
  // Apply the Gaussian material and set base color to white
  if (mesh_shape_->getEntity()) {
    mesh_shape_->getEntity()->setMaterialName("gsplat_rviz_trials/GaussianSplat");
  }
  mesh_shape_->setColor(1.0f, 1.0f, 1.0f, 1.0f);
}

Splat::~Splat() = default;

void Splat::update(Ogre::Camera * camera)
{
  if (camera && mesh_shape_) {
    mesh_shape_->setOrientation(camera->getDerivedOrientation());
  }
}

}  // namespace gsplat_rviz_trials
