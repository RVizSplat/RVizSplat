// DO NOT include GL/glew.h in this file.
// Ogre's GL3Plus header pulls in its bundled gl3w.h which conflicts with GLEW.
// By keeping this TU GLEW-free, both loaders stay in separate translation units.
#include "gsplat_rviz_trials/cuda_gl_bridge.hpp"
#include <OgreGL3PlusHardwareVertexBuffer.h>

namespace gsplat_rviz_trials
{

uint32_t getGLBufferId(Ogre::HardwareVertexBuffer * buf)
{
  return static_cast<Ogre::GL3PlusHardwareVertexBuffer *>(buf)->getGLBufferId();
}

}  // namespace gsplat_rviz_trials
