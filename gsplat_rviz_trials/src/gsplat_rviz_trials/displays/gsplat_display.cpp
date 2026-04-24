#include "gsplat_rviz_trials/displays/gsplat_display.hpp"

#include <algorithm>
#include <utility>

#include <QFileDialog>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/status_property.hpp"

#include "gsplat_rviz_trials/sorters/i_splat_sorter.hpp"
#include "gsplat_rviz_trials/sorters/sorter_factory.hpp"
#include "gsplat_rviz_trials/splat_loaders/i_splat_source.hpp"
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
  // is torn down first. The cloud owns the sorter (via its scheduler) and
  // will destroy it on the scheduler's worker thread.
  source_.reset();
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
  // A topic may already be configured; re-create the subscription.
  onTopicChanged();
}

void GsplatDisplay::onDisable()
{
  // Only drop the source if it's the ROS one — keep file data resident.
  if (dynamic_cast<RosTopicSource *>(source_.get())) {
    source_.reset();
  }
}

void GsplatDisplay::update(
  std::chrono::nanoseconds /*wall_dt*/,
  std::chrono::nanoseconds /*ros_dt*/)
{
  pollSource();
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
  if (!splat_cloud_) return;

  const QString path = splat_path_property_->getString();
  if (path.isEmpty()) {
    source_.reset();
    splat_cloud_->clear();
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

void GsplatDisplay::onAlphaThresholdChanged()
{
  if (splat_cloud_) {
    splat_cloud_->setAlphaThreshold(alpha_threshold_property_->getFloat());
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

void GsplatDisplay::rebuildSorter()
{
  if (!splat_cloud_) return;

  const auto kind = static_cast<SorterKind>(
    sorter_kind_property_ ? sorter_kind_property_->getOptionInt()
                          : static_cast<int>(SorterKind::Cuda));
  splat_cloud_->setSorter(makeSorter(kind));
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

  // SplatCloud extracts centres internally and forwards them to the sort worker.
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
