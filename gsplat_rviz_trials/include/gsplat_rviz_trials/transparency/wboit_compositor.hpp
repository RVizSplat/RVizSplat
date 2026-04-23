#ifndef GSPLAT_RVIZ_TRIALS__TRANSPARENCY__WBOIT_COMPOSITOR_HPP_
#define GSPLAT_RVIZ_TRIALS__TRANSPARENCY__WBOIT_COMPOSITOR_HPP_

namespace Ogre
{
class Viewport;
}

namespace gsplat_rviz_trials
{
namespace transparency
{

// Lifecycle helpers for the WBOIT Ogre compositor.
// Stateless — all calls are safe from any point in the display lifecycle.
namespace wboit_compositor
{

// Canonical compositor name used by the material schemes.
extern const char * const kName;

// Register the compositor definition with Ogre's CompositorManager.
// Idempotent: second calls no-op.
void ensureDefined();

// Attach the compositor to `viewport` and enable it. Safe to call repeatedly.
void enable(Ogre::Viewport * viewport);

// Disable and detach from `viewport`. Safe if nothing was attached.
void disable(Ogre::Viewport * viewport);

}  // namespace wboit_compositor
}  // namespace transparency
}  // namespace gsplat_rviz_trials

#endif  // GSPLAT_RVIZ_TRIALS__TRANSPARENCY__WBOIT_COMPOSITOR_HPP_
