#include "gsplat_rviz_trials/splat.hpp"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreMaterial.h>
#include <OgreTechnique.h>
#include <OgreVector3.h>

#include "rviz_rendering/objects/mesh_shape.hpp"

namespace gsplat_rviz_trials
{

Splat::Splat(Ogre::SceneManager * scene_manager, Ogre::SceneNode * parent_node)
{
  mesh_shape_ = std::make_unique<rviz_rendering::MeshShape>(scene_manager, parent_node);

  mesh_shape_->beginTriangles();
  
  // Define a 2x1 rectangle centered at origin
  // Vertices:
  // 0: (-1.0, -0.5, 0)
  // 1: ( 1.0, -0.5, 0)
  // 2: ( 1.0,  0.5, 0)
  // 3: (-1.0,  0.5, 0)
  
  mesh_shape_->addVertex(Ogre::Vector3(-1.0f, -0.5f, 0.0f));
  mesh_shape_->addVertex(Ogre::Vector3( 1.0f, -0.5f, 0.0f));
  mesh_shape_->addVertex(Ogre::Vector3( 1.0f,  0.5f, 0.0f));
  mesh_shape_->addVertex(Ogre::Vector3(-1.0f,  0.5f, 0.0f));
  
  // Triangle 1
  mesh_shape_->addTriangle(0, 1, 2);
  // Triangle 2
  mesh_shape_->addTriangle(0, 2, 3);
  
  mesh_shape_->endTriangles();
  
  // Set color to white
  mesh_shape_->setColor(1.0f, 1.0f, 1.0f, 1.0f);
  
  // Disable lighting to ensure pure white color as requested
  if (mesh_shape_->getMaterial()) {
    mesh_shape_->getMaterial()->setLightingEnabled(false);
  }
}

Splat::~Splat() = default;

}  // namespace gsplat_rviz_trials
