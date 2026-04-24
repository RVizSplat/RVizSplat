#ifndef GSPLAT_RVIZ_PLUGIN__TRANSPARENCY__WBOIT_COMPOSITOR_HPP_
#define GSPLAT_RVIZ_PLUGIN__TRANSPARENCY__WBOIT_COMPOSITOR_HPP_

namespace Ogre
{
class Viewport;
}

namespace gsplat_rviz_plugin
{
namespace transparency
{

// Lifecycle helpers for the WBOIT Ogre compositor.
//
// The separation between "attach / setEnabled" and "detach" is
// load-bearing: `setEnabled(false)` is safe to call mid-render because
// Ogre skips disabled compositors in the chain without mutating it.
// `detach` actually removes the compositor from the chain and must
// NOT be called during _updateRenderQueue / notifyRenderSingleObject
// — doing so leaves the chain half-torn-down and produces a blank
// viewport on the next frame. Reserve `detach` for destruction.
namespace wboit_compositor
{

// Canonical compositor name used by the material schemes.
extern const char * const kName;

// Register the compositor definition with Ogre's CompositorManager.
// Idempotent: second calls no-op.
void ensureDefined();

// Attach the compositor to `viewport` if it isn't already, and
// enable it. Idempotent — safe to call every frame. Safe mid-render.
void enable(Ogre::Viewport * viewport);

// Disable the compositor on `viewport` without removing it from the
// chain. Safe mid-render. The chain entry is kept so re-enabling is
// a pure flag flip, not a chain mutation.
void disable(Ogre::Viewport * viewport);

// Remove the compositor from `viewport`'s chain entirely. Not safe
// mid-render — call only from destruction paths where rendering on
// this viewport has stopped.
void detach(Ogre::Viewport * viewport);

}  // namespace wboit_compositor
}  // namespace transparency
}  // namespace gsplat_rviz_plugin

#endif  // GSPLAT_RVIZ_PLUGIN__TRANSPARENCY__WBOIT_COMPOSITOR_HPP_
