#include "gsplat_rviz_plugin/displays/gsplat_display.hpp"

#include <algorithm>
#include <utility>

#include <QFileDialog>
#include <QMetaObject>

#include <OgreCamera.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/status_property.hpp"
#include "rviz_common/view_controller.hpp"
#include "rviz_common/view_manager.hpp"

#include "gsplat_rviz_plugin/sorters/i_splat_sorter.hpp"
#include "gsplat_rviz_plugin/sorters/sorter_factory.hpp"
#include "gsplat_rviz_plugin/splat_loaders/ply_file_source.hpp"
#include "gsplat_rviz_plugin/splat_loaders/ros_topic_source.hpp"
#include "gsplat_rviz_plugin/splat_cloud.hpp"
#include "gsplat_rviz_plugin/splat_gpu.hpp"
#include "gsplat_rviz_plugin/transparency/wboit_compositor.hpp"

namespace gsplat_rviz_plugin
{
namespace displays
{

GsplatDisplay::GsplatDisplay()
{
  // Top-level properties — data source, mode selector, SH degree. Always
  // visible; the two load paths (file and topic) are mutually exclusive at
  // runtime but both shown so users can switch without re-opening a sub-group.
  source_mode_property_ = new rviz_common::properties::EnumProperty(
    "Source", "PLY File",
    "Where splats come from. Exactly one of the two source fields below is "
    "active at a time.",
    this, SLOT(onSourceModeChanged()), this);
  source_mode_property_->addOption("PLY File", static_cast<int>(SourceMode::File));
  source_mode_property_->addOption("Topic",    static_cast<int>(SourceMode::Topic));

  splat_path_property_ = new rviz_common::properties::FilePickerProperty(
    "Splat File", "",
    "Path to a 3DGS-format PLY file to visualize.",
    this, SLOT(onSplatPathChanged()),
    this,
    QFileDialog::ExistingFile);

  topic_property_ = new rviz_common::properties::RosTopicProperty(
    "Topic", "",
    "splat_msgs/msg/SplatArray",
    "splat_msgs/SplatArray topic to subscribe to. Each message replaces "
    "the currently displayed splats.",
    this, SLOT(onTopicChanged()), this);
  // Default mode is PLY File — hide the topic field until the user flips it.
  topic_property_->hide();

  buildAdvancedGroup();
}

// Advanced group layout (collapsed by default):
//
//   Advanced
//   ├── SH Degree
//   ├── Alpha Threshold
//   ├── Sort Backend
//   ├── Clip Box  (BoolProperty, expands to show Min/Max)
//   │   ├── Clip Min
//   │   └── Clip Max
//   └── WBOIT  (sub-group, collapsed)
//       ├── Transparency Mode
//       ├── WBOIT Weight Scale
//       ├── WBOIT Weight Exponent
//       └── WBOIT Alpha Discard
//
// Grouping rationale: these are all tuning knobs that a user may want to
// reach occasionally but don't belong in the top-level clutter. WBOIT is
// nested one level deeper because it's a niche transparency mode — the
// default Sorted path needs no tuning, so the WBOIT knobs stay out of view
// unless the user explicitly opens the sub-group.
void GsplatDisplay::buildAdvancedGroup()
{
  auto * advanced_group = new rviz_common::properties::Property(
    "Advanced", QVariant(),
    "SH degree, alpha threshold, sort backend, ROI clipping, and transparency "
    "fallback. Defaults are sensible for most scenes.",
    this);
  advanced_group->collapse();

  sh_degree_property_ = new rviz_common::properties::IntProperty(
    "SH Degree", 0,
    "Spherical harmonics degree used for view-dependent colour (0 = DC only). "
    "Lower values are faster; maximum is set by the loaded data.",
    advanced_group, SLOT(onShDegreeChanged()), this);
  sh_degree_property_->setMin(0);
  sh_degree_property_->setMax(0);

  alpha_threshold_property_ = new rviz_common::properties::FloatProperty(
    "Alpha Threshold", 0.05f,
    "Splat fragments with opacity below this value are discarded.",
    advanced_group, SLOT(onAlphaThresholdChanged()), this);
  alpha_threshold_property_->setMin(0.0f);
  alpha_threshold_property_->setMax(1.0f);

  sorter_kind_property_ = new rviz_common::properties::EnumProperty(
    "Sort Backend", "CUDA",
    "Depth-sort backend. CUDA falls back to CPU if no device is available.",
    advanced_group, SLOT(onSorterKindChanged()), this);
  sorter_kind_property_->addOption("CPU",  static_cast<int>(SorterKind::Cpu));
  sorter_kind_property_->addOption("CUDA", static_cast<int>(SorterKind::Cuda));

  // ROI clip — axis-aligned box in the scene's local frame. Useful for
  // trimming floaters, hiding ceilings on handheld captures, or isolating
  // a workspace region. The BoolProperty acts as its own parent: toggling
  // it reveals/hides the Min/Max children.
  clip_enabled_property_ = new rviz_common::properties::BoolProperty(
    "Clip Box", false,
    "Cull splats whose centre falls outside [Clip Min, Clip Max].",
    advanced_group, SLOT(onClipChanged()), this);

  clip_min_property_ = new rviz_common::properties::VectorProperty(
    "Clip Min", Ogre::Vector3(-10.0f, -10.0f, -10.0f),
    "Minimum corner of the clip AABB.",
    clip_enabled_property_, SLOT(onClipChanged()), this);

  clip_max_property_ = new rviz_common::properties::VectorProperty(
    "Clip Max", Ogre::Vector3(10.0f, 10.0f, 10.0f),
    "Maximum corner of the clip AABB.",
    clip_enabled_property_, SLOT(onClipChanged()), this);

  // WBOIT sub-group. Collapsed so the WBOIT-specific knobs only show when
  // the user opens them; Transparency Mode also lives here because
  // enabling WBOIT is the entry point to the rest of the sub-group.
  auto * wboit_group = new rviz_common::properties::Property(
    "WBOIT", QVariant(),
    "Order-independent transparency. Approximation; use when the sort is "
    "a bottleneck (very large clouds, fast camera motion, streaming splats).",
    advanced_group);
  wboit_group->collapse();

  transparency_mode_property_ = new rviz_common::properties::EnumProperty(
    "Transparency Mode", "Sorted",
    "Sorted = exact back-to-front. WBOIT = order-independent approximation.",
    wboit_group, SLOT(onTransparencyModeChanged()), this);
  transparency_mode_property_->addOption("Sorted", 0);
  transparency_mode_property_->addOption("WBOIT",  1);

  wboit_weight_scale_property_ = new rviz_common::properties::FloatProperty(
    "WBOIT Weight Scale", 5.0f,
    "Depth-discrimination strength in the WBOIT weight function.",
    wboit_group, SLOT(onWboitTuningChanged()), this);
  wboit_weight_scale_property_->setMin(0.1f);
  wboit_weight_scale_property_->setMax(50.0f);

  wboit_weight_exponent_property_ = new rviz_common::properties::FloatProperty(
    "WBOIT Weight Exponent", 2.0f,
    "Alpha-suppression power in the WBOIT weight function.",
    wboit_group, SLOT(onWboitTuningChanged()), this);
  wboit_weight_exponent_property_->setMin(0.5f);
  wboit_weight_exponent_property_->setMax(6.0f);

  wboit_alpha_discard_property_ = new rviz_common::properties::FloatProperty(
    "WBOIT Alpha Discard", 0.01f,
    "Fragment alpha cutoff in the WBOIT accumulation pass.",
    wboit_group, SLOT(onWboitTuningChanged()), this);
  wboit_alpha_discard_property_->setMin(0.0f);
  wboit_alpha_discard_property_->setMax(0.1f);
}

GsplatDisplay::~GsplatDisplay()
{
  // Detach the WBOIT compositor before the viewport goes away. Safe here
  // because destruction runs outside the render loop; see the comment
  // on transparency::wboit_compositor::detach() for why this can't run
  // during SplatCloud::_updateRenderQueue.
  if (compositor_viewport_) {
    transparency::wboit_compositor::detach(compositor_viewport_);
    compositor_viewport_ = nullptr;
    wboit_compositor_active_ = false;
  }

  // Drop the source before the cloud so any in-flight subscription callback
  // is torn down first. Bumping the generation ensures any already-queued
  // main-thread delivery is dropped rather than run during teardown.
  ++source_gen_;
  source_.reset();
  source_kind_ = SourceKind::None;
}

void GsplatDisplay::onInitialize()
{
  rviz_common::Display::onInitialize();
  splat_cloud_ = std::make_unique<SplatCloud>(scene_node_);

  rebuildSorter();

  topic_property_->initialize(context_->getRosNodeAbstraction());

  // Seed uniforms from the initial property values.  applyTransparencyMode
  // runs here too, but compositor attachment is deferred until the first
  // update() because the viewport isn't wired up yet.
  onClipChanged();
  onWboitTuningChanged();
  applyTransparencyMode();
}

void GsplatDisplay::onEnable()
{
  // If the current mode is Topic, re-create the subscription (it was torn
  // down on the previous onDisable). File mode persists across enable/disable.
  if (currentMode() == SourceMode::Topic) {
    onTopicChanged();
  }
  // Re-apply transparency mode; if WBOIT was active before disable, the
  // cloud's compositor attach is deferred to its next render pass.
  applyTransparencyMode();
}

void GsplatDisplay::onDisable()
{
  // Only drop the source if it's the ROS one — keep file data resident.
  if (source_kind_ == SourceKind::Topic) {
    ++source_gen_;
    source_.reset();
    source_kind_ = SourceKind::None;
  }
  // Disable the compositor so another display sharing this viewport
  // doesn't inherit our WBOIT passes. The user's Transparency Mode
  // property is untouched, so onEnable's applyTransparencyMode()
  // re-enables it. Leave the chain entry attached (faster re-enable
  // and avoids mid-render chain mutation).
  if (wboit_compositor_active_) {
    transparency::wboit_compositor::disable(compositor_viewport_);
    wboit_compositor_active_ = false;
  }
  if (splat_cloud_) splat_cloud_->setOitEnabled(false);
}

void GsplatDisplay::reset()
{
  rviz_common::Display::reset();
  if (splat_cloud_) splat_cloud_->clear();
  sh_degree_property_->setMax(0);
  sh_degree_property_->setValue(0);
  // Keep source_ alive: a live ROS subscription should continue delivering
  // messages after a reset, matching the pre-refactor behaviour.
}

void GsplatDisplay::onSplatPathChanged()
{
  if (currentMode() != SourceMode::File) return;
  if (!splat_cloud_) return;

  const QString path = splat_path_property_->getString();
  if (path.isEmpty()) {
    ++source_gen_;
    source_.reset();
    source_kind_ = SourceKind::None;
    splat_cloud_->clear();
    setStatus(
      rviz_common::properties::StatusProperty::Warn,
      "Splat File", "No file selected.");
    return;
  }

  installSource(
    std::make_unique<PlyFileSource>(path.toStdString()),
    SourceKind::File);
}

void GsplatDisplay::onShDegreeChanged()
{
  if (splat_cloud_) {
    splat_cloud_->setShDegree(sh_degree_property_->getInt());
  }
}

void GsplatDisplay::onAlphaThresholdChanged()
{
  if (splat_cloud_) {
    splat_cloud_->setAlphaThreshold(alpha_threshold_property_->getFloat());
  }
}

void GsplatDisplay::onTopicChanged()
{
  if (currentMode() != SourceMode::Topic) return;

  if (!isEnabled()) {
    ++source_gen_;
    source_.reset();
    source_kind_ = SourceKind::None;
    return;
  }

  const std::string topic = topic_property_->getTopicStd();
  if (topic.empty()) {
    ++source_gen_;
    source_.reset();
    source_kind_ = SourceKind::None;
    setStatus(
      rviz_common::properties::StatusProperty::Warn,
      "Topic", "No topic selected.");
    return;
  }

  auto node_abs = context_ ? context_->getRosNodeAbstraction().lock() : nullptr;
  if (!node_abs) {
    setStatus(
      rviz_common::properties::StatusProperty::Error,
      "Topic", "RViz ROS node unavailable.");
    return;
  }

  installSource(
    std::make_unique<RosTopicSource>(node_abs->get_raw_node(), topic),
    SourceKind::Topic);
}

void GsplatDisplay::onSorterKindChanged()
{
  rebuildSorter();
}

GsplatDisplay::SourceMode GsplatDisplay::currentMode() const
{
  return source_mode_property_
    ? static_cast<SourceMode>(source_mode_property_->getOptionInt())
    : SourceMode::File;
}

void GsplatDisplay::onSourceModeChanged()
{
  // Tear down the outgoing source and any visible state tied to it.
  ++source_gen_;
  source_.reset();
  source_kind_ = SourceKind::None;
  if (splat_cloud_) splat_cloud_->clear();
  sh_degree_property_->setMax(0);
  sh_degree_property_->setValue(0);

  const auto mode = currentMode();
  splat_path_property_->setHidden(mode != SourceMode::File);
  topic_property_->setHidden(mode != SourceMode::Topic);

  // Drop any stale status from the other source so only the active one's
  // status is visible in the panel.
  deleteStatus(mode == SourceMode::File ? "Topic" : "Splat File");

  // Activate the newly-selected source if its field already has a value.
  if (mode == SourceMode::File) {
    onSplatPathChanged();
  } else {
    onTopicChanged();
  }
}

void GsplatDisplay::onClipChanged()
{
  if (!splat_cloud_) return;
  splat_cloud_->setClipEnabled(clip_enabled_property_->getBool());
  splat_cloud_->setClipBox(
    clip_min_property_->getVector(),
    clip_max_property_->getVector());
  if (context_) context_->queueRender();
}

void GsplatDisplay::onTransparencyModeChanged()
{
  applyTransparencyMode();
}

void GsplatDisplay::onWboitTuningChanged()
{
  if (!splat_cloud_) return;
  splat_cloud_->setWboitWeightScale(wboit_weight_scale_property_->getFloat());
  splat_cloud_->setWboitWeightExponent(wboit_weight_exponent_property_->getFloat());
  splat_cloud_->setWboitAlphaDiscard(wboit_alpha_discard_property_->getFloat());
  if (context_) context_->queueRender();
}

void GsplatDisplay::applyTransparencyMode()
{
  const bool wboit = (transparency_mode_property_ &&
                      transparency_mode_property_->getOptionInt() == 1);

  // Update the cloud's queue-group gate (50 Sorted / 95 WBOIT).
  if (splat_cloud_) splat_cloud_->setOitEnabled(wboit);

  // Toggle the compositor on the current viewport. Running this on the
  // Qt main thread (we're inside a property slot) means Ogre is *not*
  // mid-frame, so chain mutations are safe. The first attach also
  // happens here once a viewport is available.
  if (auto * vp = resolveViewport()) {
    if (wboit) {
      transparency::wboit_compositor::ensureDefined();
      transparency::wboit_compositor::enable(vp);
      compositor_viewport_ = vp;
      wboit_compositor_active_ = true;
    } else if (wboit_compositor_active_) {
      transparency::wboit_compositor::disable(compositor_viewport_);
      wboit_compositor_active_ = false;
    }
  }
  // If vp == nullptr the viewport isn't wired up yet (called from very
  // early onInitialize); the next applyTransparencyMode call (onEnable
  // or a property slot) will find it and attach.

  if (context_) context_->queueRender();
}

Ogre::Viewport * GsplatDisplay::resolveViewport() const
{
  if (!context_) return nullptr;
  auto * vm = context_->getViewManager();
  if (!vm) return nullptr;
  auto * vc = vm->getCurrent();
  if (!vc) return nullptr;
  auto * cam = vc->getCamera();
  if (!cam) return nullptr;
  return cam->getViewport();
}

void GsplatDisplay::rebuildSorter()
{
  if (!splat_cloud_) return;

  const auto kind = static_cast<SorterKind>(
    sorter_kind_property_ ? sorter_kind_property_->getOptionInt()
                          : static_cast<int>(SorterKind::Cuda));
  splat_cloud_->setSorter(makeSorter(kind));
}

void GsplatDisplay::installSource(
  std::unique_ptr<ISplatSource> source, SourceKind kind)
{
  // Replace the current source atomically from the main thread's perspective:
  // bump the generation first so any queued delivery from the outgoing source
  // is dropped, then tear it down, then wire the new one.
  const uint64_t gen = ++source_gen_;
  source_.reset();
  source_kind_ = kind;
  source_ = std::move(source);

  source_->start(
    [this, kind, gen](LoadResult r) {
      // AutoConnection: direct call on the display's thread (PLY case),
      // queued post from the ROS executor thread otherwise.
      QMetaObject::invokeMethod(
        this,
        [this, kind, gen, r = std::move(r)]() mutable {
          onLoadResult(std::move(r), kind, gen);
        });
    });
}

void GsplatDisplay::onLoadResult(
  LoadResult result, SourceKind kind, uint64_t gen)
{
  if (gen != source_gen_ || !splat_cloud_) return;

  const char * status_key =
    (kind == SourceKind::Topic) ? "Topic" : "Splat File";

  if (!result.ok()) {
    setStatus(
      rviz_common::properties::StatusProperty::Error,
      status_key, QString::fromStdString(result.error));
    return;
  }

  const int count = static_cast<int>(result.splats.size());
  const int sh_degree = result.sh_degree;

  // SplatCloud extracts centres internally and forwards them to the sort worker.
  splat_cloud_->setSplats(std::move(result.splats), sh_degree);

  sh_degree_property_->setMax(sh_degree);
  if (kind == SourceKind::Topic) {
    const int current = sh_degree_property_->getInt();
    if (current > sh_degree) sh_degree_property_->setValue(sh_degree);
  } else {
    sh_degree_property_->setValue(std::min(1, sh_degree));
  }

  setStatus(
    rviz_common::properties::StatusProperty::Ok,
    status_key,
    kind == SourceKind::Topic
      ? QString("Received %1 gaussians (SH degree %2)").arg(count).arg(sh_degree)
      : QString("Loaded %1 gaussians").arg(count));
}

}  // namespace displays
}  // namespace gsplat_rviz_plugin

PLUGINLIB_EXPORT_CLASS(
  gsplat_rviz_plugin::displays::GsplatDisplay,
  rviz_common::Display)
