#include "gsplat_rviz_trials/transparency/wboit_compositor.hpp"

#include <OgreResource.h>
#include <OgreCompositor.h>
#include <OgreCompositorManager.h>
#include <OgreCompositionPass.h>
#include <OgreCompositionTargetPass.h>
#include <OgreCompositionTechnique.h>
#include <OgrePixelFormat.h>
#include <OgreViewport.h>

#include <iostream>  // TODO(wboit-debug): remove after MRT issue diagnosed

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

  // --- Textures -----------------------------------------------------------
  //
  // scene_tex : opaque scene target (pass 1 output, pass 3 input).
  // wboit_mrt : two-attachment MRT written in a single splat-raster pass.
  //             [0] accum      (RGBA16F) — Σ wᵢαᵢRGBᵢ in .rgb, Σ wᵢαᵢ in .a
  //             [1] reveal_log (RGBA16F) — Σ -ln(1-αᵢ) in .r
  //
  // Both MRT attachments use identical additive ONE/ONE blending (set by the
  // wboit_accum material scheme), which is why MRT fusion is legal without
  // per-attachment blend state (GL 4.0+ glBlendFunci not required).
  auto * scene_td = tech->createTextureDefinition("scene_tex");
  scene_td->formatList.push_back(Ogre::PF_A8R8G8B8);

  auto * mrt_td = tech->createTextureDefinition("wboit_mrt");
  mrt_td->formatList.push_back(Ogre::PF_FLOAT16_RGBA);  // attachment 0: accum
  mrt_td->formatList.push_back(Ogre::PF_FLOAT16_RGBA);  // attachment 1: reveal_log

  // --- Pass 1: opaque scene ----------------------------------------------
  //
  // Explicit depth clear — scene_tex shares the viewport's depth buffer with
  // the MRT pass, so stale splat depth from frame N-1 would reject opaque
  // geometry on frame N without this clear.
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

  // --- Pass 2: splats → MRT (accum + reveal_log) --------------------------
  //
  // Both attachments clear to 0:
  //   accum      : additive sum starts at 0.
  //   reveal_log : Σ -ln(1-αᵢ) with zero fragments is 0 →
  //                resolve sees exp(0)=1 (fully transparent, no splats).
  //
  // Depth test enabled (set on the material pass) so opaque pass-1 geometry
  // occludes transparent splats; depth writes disabled so accum order
  // doesn't matter.
  {
    auto * tp = tech->createTargetPass();
    tp->setInputMode(Ogre::CompositionTargetPass::IM_NONE);
    tp->setOutputName("wboit_mrt");
    tp->setMaterialScheme("wboit_accum");
    auto * cl = tp->createPass(Ogre::CompositionPass::PT_CLEAR);
    cl->setClearColour(Ogre::ColourValue(0, 0, 0, 0));
    cl->setClearBuffers(Ogre::FBT_COLOUR);
    auto * rp = tp->createPass(Ogre::CompositionPass::PT_RENDERSCENE);
    rp->setFirstRenderQueue(95);
    rp->setLastRenderQueue(95);
  }

  // --- Pass 3: resolve over opaque scene via full-screen quad ------------
  //
  // The MRT attachments are consumed as two separate textures via the
  // mrtIndex parameter of CompositionPass::setInput.
  {
    auto * tp = tech->getOutputTargetPass();
    tp->setInputMode(Ogre::CompositionTargetPass::IM_NONE);
    auto * q = tp->createPass(Ogre::CompositionPass::PT_RENDERQUAD);
    q->setMaterialName("gsplat_rviz_trials/WboitResolve");
    q->setInput(0, "scene_tex");
    q->setInput(1, "wboit_mrt", 0);   // accum attachment
    q->setInput(2, "wboit_mrt", 1);   // reveal_log attachment
  }

  comp->load();
  std::cerr << "[WBOIT-DBG] ensureDefined OK isLoaded="
            << comp->isLoaded() << std::endl;
}

void enable(Ogre::Viewport * viewport)
{
  if (!viewport) return;
  auto & mgr = Ogre::CompositorManager::getSingleton();
  try {
    auto * inst = mgr.addCompositor(viewport, kName);
    if (!inst) {
      std::cerr << "[WBOIT-DBG] addCompositor returned null — compositor not registered?"
                << std::endl;
      return;
    }
    mgr.setCompositorEnabled(viewport, kName, true);
    std::cerr << "[WBOIT-DBG] compositor enabled on viewport "
              << viewport << std::endl;
  } catch (const Ogre::Exception & e) {
    std::cerr << "[WBOIT-DBG] Ogre::Exception in enable(): "
              << e.getFullDescription() << std::endl;
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
