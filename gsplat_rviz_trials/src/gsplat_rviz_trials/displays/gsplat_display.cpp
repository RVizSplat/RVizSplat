#include "gsplat_rviz_trials/displays/gsplat_display.hpp"

#include <algorithm>
#include <utility>

#include <QFileDialog>
#include <QMetaObject>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/status_property.hpp"

#include "gsplat_rviz_trials/sorters/i_splat_sorter.hpp"
#include "gsplat_rviz_trials/sorters/sorter_factory.hpp"
#include "gsplat_rviz_trials/splat_loaders/ply_file_source.hpp"
#include "gsplat_rviz_trials/splat_loaders/ros_topic_source.hpp"
#include "gsplat_rviz_trials/splat_cloud.hpp"
#include "gsplat_rviz_trials/splat_gpu.hpp"

namespace gsplat_rviz_trials
{
namespace displays
{

GsplatDisplay::GsplatDisplay()
{
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

  sh_degree_property_ = new rviz_common::properties::IntProperty(
    "SH Degree", 0,
    "Spherical harmonics degree used for view-dependent colour (0 = DC only). "
    "Lower values are faster; maximum is set by the loaded data.",
    this, SLOT(onShDegreeChanged()), this);
  sh_degree_property_->setMin(0);
  sh_degree_property_->setMax(0);

  alpha_threshold_property_ = new rviz_common::properties::FloatProperty(
    "Alpha Threshold", 0.05f,
    "Splat fragments with opacity below this value are discarded.",
    this, SLOT(onAlphaThresholdChanged()), this);
  alpha_threshold_property_->setMin(0.0f);
  alpha_threshold_property_->setMax(1.0f);

  sorter_kind_property_ = new rviz_common::properties::EnumProperty(
    "Sort Backend", "CUDA",
    "Depth-sort backend. CUDA falls back to CPU if no device is available.",
    this, SLOT(onSorterKindChanged()), this);
  sorter_kind_property_->addOption("CPU",  static_cast<int>(SorterKind::Cpu));
  sorter_kind_property_->addOption("CUDA", static_cast<int>(SorterKind::Cuda));
}

GsplatDisplay::~GsplatDisplay()
{
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
}

void GsplatDisplay::onEnable()
{
  // If the current mode is Topic, re-create the subscription (it was torn
  // down on the previous onDisable). File mode persists across enable/disable.
  if (currentMode() == SourceMode::Topic) {
    onTopicChanged();
  }
}

void GsplatDisplay::onDisable()
{
  // Only drop the source if it's the ROS one — keep file data resident.
  if (source_kind_ == SourceKind::Topic) {
    ++source_gen_;
    source_.reset();
    source_kind_ = SourceKind::None;
  }
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
}  // namespace gsplat_rviz_trials

PLUGINLIB_EXPORT_CLASS(
  gsplat_rviz_trials::displays::GsplatDisplay,
  rviz_common::Display)
