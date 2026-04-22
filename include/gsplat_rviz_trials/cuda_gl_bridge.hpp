#pragma once
#include <cstdint>

namespace Ogre { class HardwareVertexBuffer; }

namespace gsplat_rviz_trials
{

// Returns the raw OpenGL buffer object ID for an Ogre vertex buffer.
// Implemented in a separate TU that includes the GL3Plus header without GLEW
// to avoid symbol conflicts between GLEW and Ogre's bundled gl3w.
uint32_t getGLBufferId(Ogre::HardwareVertexBuffer * buf);

}  // namespace gsplat_rviz_trials
