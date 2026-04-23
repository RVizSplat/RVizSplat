#include "gsplat_rviz_trials/displays/gsplat_display.hpp"

#include <algorithm>
#include <utility>

#include <QFileDialog>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
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

  reference_frame_property_ = new rviz_common::properties::TfFrameProperty(
    "Reference Frame",
    rviz_common::properties::TfFrameProperty::FIXED_FRAME_STRING,
    "TF frame the splat cloud is anchored to.  Each tick we look up the "
    "frame's pose in Fixed Frame and plant the cloud there.  Use any robot-"
    "attached frame to see a captured scene move with the robot, or leave "
    "at <Fixed Frame> for a world-locked reconstruction.",
    this, nullptr, true);

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

  // ROI clip — axis-aligned box in the Reference Frame's coordinates.
  // Useful daily for trimming floaters, hiding ceilings on handheld captures,
  // or isolating a workspace region.
  clip_enabled_property_ = new rviz_common::properties::BoolProperty(
    "Clip Box", false,
    "Cull splats whose centre falls outside [Clip Min, Clip Max].",
    this, SLOT(onClipChanged()), this);

  clip_min_property_ = new rviz_common::properties::VectorProperty(
    "Clip Min", Ogre::Vector3(-10.0f, -10.0f, -10.0f),
    "Minimum corner of the clip AABB, in Reference Frame coordinates.",
    clip_enabled_property_, SLOT(onClipChanged()), this);

  clip_max_property_ = new rviz_common::properties::VectorProperty(
    "Clip Max", Ogre::Vector3(10.0f, 10.0f, 10.0f),
    "Maximum corner of the clip AABB, in Reference Frame coordinates.",
    clip_enabled_property_, SLOT(onClipChanged()), this);
}

GsplatDisplay::~GsplatDisplay()
{
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
  reference_frame_property_->setFrameManager(context_->getFrameManager());

  // Seed the clip uniforms from the initial property values.
  onClipChanged();
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

  // Pose the scene node at the Reference Frame's current pose in Fixed Frame.
  if (scene_node_ && context_ && context_->getFrameManager()) {
    const std::string frame = reference_frame_property_->getFrameStd();
    Ogre::Vector3    position;
    Ogre::Quaternion orientation;
    if (context_->getFrameManager()->getTransform(frame, position, orientation)) {
      scene_node_->setPosition(position);
      scene_node_->setOrientation(orientation);
      setStatus(
        rviz_common::properties::StatusProperty::Ok,
        "Reference Frame", "Transform OK");
    } else {
      setStatus(
        rviz_common::properties::StatusProperty::Warn,
        "Reference Frame",
        QString("Frame '%1' not available in TF yet.")
          .arg(QString::fromStdString(frame)));
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
