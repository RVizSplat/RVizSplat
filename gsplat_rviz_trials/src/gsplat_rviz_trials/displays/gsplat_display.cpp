#include "gsplat_rviz_trials/displays/gsplat_display.hpp"

#include <algorithm>
#include <utility>

#include <QFileDialog>

#include <OgreCamera.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/status_property.hpp"
#include "rviz_common/view_controller.hpp"
#include "rviz_common/view_manager.hpp"

#include "gsplat_rviz_trials/sorters/i_splat_sorter.hpp"
#include "gsplat_rviz_trials/sorters/sorter_factory.hpp"
#include "gsplat_rviz_trials/splat_loaders/i_splat_source.hpp"
#include "gsplat_rviz_trials/splat_loaders/ply_file_source.hpp"
#include "gsplat_rviz_trials/splat_loaders/ros_topic_source.hpp"
#include "gsplat_rviz_trials/splat_cloud.hpp"
#include "gsplat_rviz_trials/splat_gpu.hpp"
#include "gsplat_rviz_trials/transparency/wboit_compositor.hpp"

namespace gsplat_rviz_trials
{
namespace displays
{

GsplatDisplay::GsplatDisplay()
{
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

  sh_degree_property_ = new rviz_common::properties::IntProperty(
    "SH Degree", 0,
    "Spherical harmonics degree used for view-dependent colour (0 = DC only). "
    "Lower values are faster; maximum is set by the loaded data.",
    this, SLOT(onShDegreeChanged()), this);
  sh_degree_property_->setMin(0);
  sh_degree_property_->setMax(0);

  sorter_kind_property_ = new rviz_common::properties::EnumProperty(
    "Sort Backend", "CUDA",
    "Depth-sort backend. Auto picks CUDA when a device is available, else CPU.",
    this, SLOT(onSorterKindChanged()), this);
  sorter_kind_property_->addOption("Auto", static_cast<int>(SorterKind::Auto));
  sorter_kind_property_->addOption("CPU",  static_cast<int>(SorterKind::Cpu));
  sorter_kind_property_->addOption("CUDA", static_cast<int>(SorterKind::Cuda));

  // ROI clip — axis-aligned box in the scene's local frame.
  // Useful daily for trimming floaters, hiding ceilings on handheld captures,
  // or isolating a workspace region.
  clip_enabled_property_ = new rviz_common::properties::BoolProperty(
    "Clip Box", false,
    "Cull splats whose centre falls outside [Clip Min, Clip Max].",
    this, SLOT(onClipChanged()), this);

  clip_min_property_ = new rviz_common::properties::VectorProperty(
    "Clip Min", Ogre::Vector3(-10.0f, -10.0f, -10.0f),
    "Minimum corner of the clip AABB.",
    clip_enabled_property_, SLOT(onClipChanged()), this);

  clip_max_property_ = new rviz_common::properties::VectorProperty(
    "Clip Max", Ogre::Vector3(10.0f, 10.0f, 10.0f),
    "Maximum corner of the clip AABB.",
    clip_enabled_property_, SLOT(onClipChanged()), this);

  // Advanced: transparency fallback.  Sorted is the exact, default path;
  // WBOIT is an order-independent approximation kept here for cases where
  // the sort is a bottleneck (very large CPU-sort clouds, fast camera
  // motion, streaming splats).  Collapsed by default — most users stay on
  // Sorted and never open this.
  auto * advanced_group = new rviz_common::properties::Property(
    "Advanced", QVariant(),
    "Fallback transparency options. Sorted is the recommended default.",
    this);
  advanced_group->collapse();

  transparency_mode_property_ = new rviz_common::properties::EnumProperty(
    "Transparency Mode", "Sorted",
    "Sorted = exact back-to-front. WBOIT = order-independent approximation.",
    advanced_group, SLOT(onTransparencyModeChanged()), this);
  transparency_mode_property_->addOption("Sorted", 0);
  transparency_mode_property_->addOption("WBOIT",  1);

  wboit_weight_scale_property_ = new rviz_common::properties::FloatProperty(
    "WBOIT Weight Scale", 5.0f,
    "Depth-discrimination strength in the WBOIT weight function.",
    advanced_group, SLOT(onWboitTuningChanged()), this);
  wboit_weight_scale_property_->setMin(0.1f);
  wboit_weight_scale_property_->setMax(50.0f);

  wboit_weight_exponent_property_ = new rviz_common::properties::FloatProperty(
    "WBOIT Weight Exponent", 2.0f,
    "Alpha-suppression power in the WBOIT weight function.",
    advanced_group, SLOT(onWboitTuningChanged()), this);
  wboit_weight_exponent_property_->setMin(0.5f);
  wboit_weight_exponent_property_->setMax(6.0f);

  wboit_alpha_discard_property_ = new rviz_common::properties::FloatProperty(
    "WBOIT Alpha Discard", 0.01f,
    "Fragment alpha cutoff in the WBOIT accumulation pass.",
    advanced_group, SLOT(onWboitTuningChanged()), this);
  wboit_alpha_discard_property_->setMin(0.0f);
  wboit_alpha_discard_property_->setMax(0.1f);
}

GsplatDisplay::~GsplatDisplay()
{
  // Detach the compositor before the viewport goes away.
  if (wboit_compositor_active_) {
    transparency::wboit_compositor::disable(compositor_viewport_);
    wboit_compositor_active_ = false;
  }
  // Drop the source before the sorter/cloud so any in-flight subscription
  // callback is torn down first.
  source_.reset();
  // Detach sorter from cloud before either is destroyed.
  if (splat_cloud_) splat_cloud_->setSorter(nullptr);
  sorter_.reset();
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
  // A topic may already be configured; re-create the subscription.
  onTopicChanged();
  // Re-attach the compositor if WBOIT was active before the display was
  // toggled off.
  applyTransparencyMode();
}

void GsplatDisplay::onDisable()
{
  // Only drop the source if it's the ROS one — keep file data resident.
  if (dynamic_cast<RosTopicSource *>(source_.get())) {
    source_.reset();
  }
  // Detach the compositor while disabled so another display doesn't inherit
  // our WBOIT passes on its viewport.
  if (wboit_compositor_active_) {
    transparency::wboit_compositor::disable(compositor_viewport_);
    wboit_compositor_active_ = false;
  }
}

void GsplatDisplay::update(
  std::chrono::nanoseconds /*wall_dt*/,
  std::chrono::nanoseconds /*ros_dt*/)
{
  pollSource();

  // Capture the viewport on the first frame it's available so the transparency
  // mode can attach the compositor.  Defers to the first update() because the
  // compositor chain needs a live viewport that isn't wired up at onInitialize().
  if (!compositor_viewport_ && context_ && context_->getViewManager()) {
    if (auto * vc = context_->getViewManager()->getCurrent()) {
      if (auto * cam = vc->getCamera()) {
        compositor_viewport_ = cam->getViewport();
        if (compositor_viewport_) {
          transparency::wboit_compositor::ensureDefined();
          applyTransparencyMode();
        }
      }
    }
  }

}

void GsplatDisplay::reset()
{
  rviz_common::Display::reset();
  if (splat_cloud_) splat_cloud_->clear();
  if (sorter_)      sorter_->reset();
  centers_cache_.clear();
  sh_degree_property_->setMax(0);
  sh_degree_property_->setValue(0);
  // Keep source_ alive: a live ROS subscription should continue delivering
  // messages after a reset, matching the pre-refactor behaviour.
}

void GsplatDisplay::onSplatPathChanged()
{
  if (!splat_cloud_) return;

  const QString path = splat_path_property_->getString();
  if (path.isEmpty()) {
    source_.reset();
    splat_cloud_->clear();
    if (sorter_) sorter_->reset();
    centers_cache_.clear();
    setStatus(
      rviz_common::properties::StatusProperty::Warn,
      "Splat File", "No file selected.");
    return;
  }

  source_ = std::make_unique<PlyFileSource>(path.toStdString());
  // PlyFileSource loads synchronously in its ctor; poll immediately so the
  // user sees errors/data without waiting for the next update() tick.
  pollSource();
}

void GsplatDisplay::onShDegreeChanged()
{
  if (splat_cloud_) {
    splat_cloud_->setShDegree(sh_degree_property_->getInt());
  }
}

void GsplatDisplay::onTopicChanged()
{
  if (!isEnabled()) {
    source_.reset();
    return;
  }

  const std::string topic = topic_property_->getTopicStd();
  if (topic.empty()) {
    source_.reset();
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

  auto ros_source = std::make_unique<RosTopicSource>(
    node_abs->get_raw_node(), topic);
  if (!ros_source->initError().empty()) {
    setStatus(
      rviz_common::properties::StatusProperty::Error,
      "Topic", QString::fromStdString(ros_source->initError()));
    return;
  }

  source_ = std::move(ros_source);
  setStatus(
    rviz_common::properties::StatusProperty::Ok,
    "Topic",
    QString("Subscribed to %1").arg(QString::fromStdString(topic)));
}

void GsplatDisplay::onSorterKindChanged()
{
  rebuildSorter();
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

  if (splat_cloud_) splat_cloud_->setOitEnabled(wboit);

  // Compositor attach/detach (no-ops until the viewport is captured).
  if (compositor_viewport_) {
    if (wboit && !wboit_compositor_active_) {
      transparency::wboit_compositor::enable(compositor_viewport_);
      wboit_compositor_active_ = true;
    } else if (!wboit && wboit_compositor_active_) {
      transparency::wboit_compositor::disable(compositor_viewport_);
      wboit_compositor_active_ = false;
    }
  }
  if (context_) context_->queueRender();
}

void GsplatDisplay::rebuildSorter()
{
  if (splat_cloud_) splat_cloud_->setSorter(nullptr);

  const auto kind = static_cast<SorterKind>(
    sorter_kind_property_ ? sorter_kind_property_->getOptionInt()
                          : static_cast<int>(SorterKind::Auto));
  sorter_ = makeSorter(kind);

  if (!centers_cache_.empty()) {
    sorter_->uploadCenters(centers_cache_);
  }
  if (splat_cloud_) splat_cloud_->setSorter(sorter_.get());
}

void GsplatDisplay::pollSource()
{
  if (!source_ || !splat_cloud_) return;

  auto result = source_->poll();
  if (!result) return;

  // Determine status key from source type so file/topic statuses don't clobber each other.
  const bool from_topic = (dynamic_cast<RosTopicSource *>(source_.get()) != nullptr);
  const char * status_key = from_topic ? "Topic" : "Splat File";

  if (!result->ok()) {
    setStatus(
      rviz_common::properties::StatusProperty::Error,
      status_key, QString::fromStdString(result->error));
    return;
  }

  const int count = static_cast<int>(result->splats.size());
  const int sh_degree = result->sh_degree;

  centers_cache_.resize(result->splats.size());
  for (size_t i = 0; i < result->splats.size(); ++i) {
    const auto & s = result->splats[i];
    centers_cache_[i] = Ogre::Vector3(s.center[0], s.center[1], s.center[2]);
  }

  if (sorter_) sorter_->uploadCenters(centers_cache_);
  splat_cloud_->setSplats(std::move(result->splats), sh_degree);

  sh_degree_property_->setMax(sh_degree);
  if (from_topic) {
    const int current = sh_degree_property_->getInt();
    if (current > sh_degree) sh_degree_property_->setValue(sh_degree);
  } else {
    sh_degree_property_->setValue(std::min(1, sh_degree));
  }

  setStatus(
    rviz_common::properties::StatusProperty::Ok,
    status_key,
    from_topic
      ? QString("Received %1 gaussians (SH degree %2)").arg(count).arg(sh_degree)
      : QString("Loaded %1 gaussians").arg(count));
}

}  // namespace displays
}  // namespace gsplat_rviz_trials

PLUGINLIB_EXPORT_CLASS(
  gsplat_rviz_trials::displays::GsplatDisplay,
  rviz_common::Display)
