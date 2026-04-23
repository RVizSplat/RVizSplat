#include "gsplat_rviz_trials/transparency/wboit_compositor.hpp"

#include <OgreResource.h>
#include <OgreCompositor.h>
#include <OgreCompositorManager.h>
#include <OgreCompositionPass.h>
#include <OgreCompositionTargetPass.h>
#include <OgreCompositionTechnique.h>
#include <OgrePixelFormat.h>
#include <OgreViewport.h>

namespace gsplat_rviz_trials
{
namespace transparency
{
namespace wboit_compositor
{

const char * const kName = "gsplat_rviz_trials/WBOIT";

void ensureDefined()
{
  auto & mgr = Ogre::CompositorManager::getSingleton();
  if (mgr.getByName(kName)) return;

  auto comp = mgr.create(kName,
    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  auto * tech = comp->createTechnique();

  auto * scene_td = tech->createTextureDefinition("scene_tex");
  scene_td->formatList.push_back(Ogre::PF_A8R8G8B8);

  auto * accum_td = tech->createTextureDefinition("accum_tex");
  accum_td->formatList.push_back(Ogre::PF_FLOAT16_RGBA);

  auto * reveal_td = tech->createTextureDefinition("reveal_tex");
  reveal_td->formatList.push_back(Ogre::PF_FLOAT16_RGBA);

  // Pass 1: opaque scene. Explicit depth clear — all three targets share the
  // viewport's depth buffer, so stale splat depth from frame N-1 would reject
  // opaque geometry on frame N without this.
  {
    auto * tp = tech->createTargetPass();
    tp->setInputMode(Ogre::CompositionTargetPass::IM_NONE);
    tp->setOutputName("scene_tex");
    auto * cl = tp->createPass(Ogre::CompositionPass::PT_CLEAR);
    cl->setClearColour(Ogre::ColourValue(0, 0, 0, 0));
    cl->setClearDepth(1.0f);
    cl->setClearBuffers(Ogre::FBT_COLOUR | Ogre::FBT_DEPTH);
    auto * rp = tp->createPass(Ogre::CompositionPass::PT_RENDERSCENE);
    rp->setLastRenderQueue(94);
  }

  // Pass 2: accumulation (additive).
  {
    auto * tp = tech->createTargetPass();
    tp->setInputMode(Ogre::CompositionTargetPass::IM_NONE);
    tp->setOutputName("accum_tex");
    tp->setMaterialScheme("wboit_accum");
    auto * cl = tp->createPass(Ogre::CompositionPass::PT_CLEAR);
    cl->setClearColour(Ogre::ColourValue(0, 0, 0, 0));
    cl->setClearBuffers(Ogre::FBT_COLOUR);
    auto * rp = tp->createPass(Ogre::CompositionPass::PT_RENDERSCENE);
    rp->setFirstRenderQueue(95);
    rp->setLastRenderQueue(95);
  }

  // Pass 3: revealage. Cleared to 1; each fragment multiplies by (1-alpha).
  {
    auto * tp = tech->createTargetPass();
    tp->setInputMode(Ogre::CompositionTargetPass::IM_NONE);
    tp->setOutputName("reveal_tex");
    tp->setMaterialScheme("wboit_reveal");
    auto * cl = tp->createPass(Ogre::CompositionPass::PT_CLEAR);
    cl->setClearColour(Ogre::ColourValue(1, 1, 1, 1));
    cl->setClearBuffers(Ogre::FBT_COLOUR);
    auto * rp = tp->createPass(Ogre::CompositionPass::PT_RENDERSCENE);
    rp->setFirstRenderQueue(95);
    rp->setLastRenderQueue(95);
  }

  // Pass 4: resolve over opaque scene via a full-screen quad.
  {
    auto * tp = tech->getOutputTargetPass();
    tp->setInputMode(Ogre::CompositionTargetPass::IM_NONE);
    auto * q = tp->createPass(Ogre::CompositionPass::PT_RENDERQUAD);
    q->setMaterialName("gsplat_rviz_trials/WboitResolve");
    q->setInput(0, "scene_tex");
    q->setInput(1, "accum_tex");
    q->setInput(2, "reveal_tex");
  }

  comp->load();
}

void enable(Ogre::Viewport * viewport)
{
  if (!viewport) return;
  auto & mgr = Ogre::CompositorManager::getSingleton();
  try {
    if (!mgr.addCompositor(viewport, kName)) return;
    mgr.setCompositorEnabled(viewport, kName, true);
  } catch (const Ogre::Exception &) {
    // Swallow — caller can surface the failure via its own status path.
  }
}

void disable(Ogre::Viewport * viewport)
{
  if (!viewport) return;
  auto & mgr = Ogre::CompositorManager::getSingleton();
  try { mgr.setCompositorEnabled(viewport, kName, false); } catch (...) {}
  try { mgr.removeCompositor(viewport, kName); } catch (...) {}
}

}  // namespace wboit_compositor
}  // namespace transparency
}  // namespace gsplat_rviz_trials
