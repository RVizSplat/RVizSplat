#include "gsplat_rviz_trials/displays/gsplat_display.hpp"

#include <algorithm>
#include <cstdint>

#include <QFileDialog>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/status_property.hpp"

#include "gsplat_rviz_trials/ply_loader.hpp"
#include "gsplat_rviz_trials/splat_cloud.hpp"
#include "gsplat_rviz_trials/splat_gpu.hpp"

namespace gsplat_rviz_trials
{
namespace displays
{

namespace
{

// Convert a SplatArray message into the plugin's GPU-ready struct layout.
//
// Wire format (produced by splat_publisher/ply_splat_publisher.py):
//   pose.pose.pose.position            →  center[3]
//   opacity (uint8, 0..255)            →  alpha = opacity / 255
//   pose.pose.covariance (6x6 r-major) →  covA/covB from top-left 3x3
//   spherical_harmonics (coef-major,
//     RGB-interleaved: [R0,G0,B0,R1,…])→  sh[k][0..2] = RGB_k
//
// Assumes publisher conformance: uniform harmonics_deg across all splats,
// deg ≤ 3 (SplatGPU::sh holds 16 coefficients), and spherical_harmonics
// length == 3·(deg+1)².
std::vector<SplatGPU> splatsFromMessage(
  const splat_msgs::msg::SplatArray & msg, int & sh_degree_out)
{
  sh_degree_out = msg.splats.empty() ? 0 : static_cast<int>(msg.splats.front().harmonics_deg);
  const int num_coefs = (sh_degree_out + 1) * (sh_degree_out + 1);

  std::vector<SplatGPU> out;
  out.reserve(msg.splats.size());

  for (const auto & s : msg.splats) {
    SplatGPU g{};

    const auto & p = s.pose.pose.pose.position;
    g.center[0] = static_cast<float>(p.x);
    g.center[1] = static_cast<float>(p.y);
    g.center[2] = static_cast<float>(p.z);

    g.alpha = static_cast<float>(s.opacity) / 255.0f;

    // 6x6 row-major → top-left 3x3
    const auto & cov = s.pose.pose.covariance;
    g.covA[0] = static_cast<float>(cov[0]);   // (0,0)
    g.covA[1] = static_cast<float>(cov[1]);   // (0,1)
    g.covA[2] = static_cast<float>(cov[2]);   // (0,2)
    g.covB[0] = static_cast<float>(cov[7]);   // (1,1)
    g.covB[1] = static_cast<float>(cov[8]);   // (1,2)
    g.covB[2] = static_cast<float>(cov[14]);  // (2,2)

    for (int k = 0; k < num_coefs; ++k) {
      g.sh[k][0] = s.spherical_harmonics[3 * k + 0];
      g.sh[k][1] = s.spherical_harmonics[3 * k + 1];
      g.sh[k][2] = s.spherical_harmonics[3 * k + 2];
      g.sh[k][3] = 0.0f;
    }

    out.push_back(g);
  }
  return out;
}

}  // namespace

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
}

GsplatDisplay::~GsplatDisplay()
{
  // Tear the subscription down first so no callback races with the
  // destruction of pending_msg_mutex_/pending_msg_.
  unsubscribe();
}

void GsplatDisplay::onInitialize()
{
  rviz_common::Display::onInitialize();
  splat_cloud_ = std::make_unique<SplatCloud>(scene_node_);

  topic_property_->initialize(context_->getRosNodeAbstraction());
}

void GsplatDisplay::onEnable()
{
  subscribe();
}

void GsplatDisplay::onDisable()
{
  unsubscribe();
}

void GsplatDisplay::update(
  std::chrono::nanoseconds /*wall_dt*/,
  std::chrono::nanoseconds /*ros_dt*/)
{
  applyPendingMessage();
}

void GsplatDisplay::reset()
{
  rviz_common::Display::reset();
  if (splat_cloud_) {
    splat_cloud_->clear();
  }
  {
    std::lock_guard<std::mutex> lock(pending_msg_mutex_);
    pending_msg_.reset();
  }
  sh_degree_property_->setMax(0);
  sh_degree_property_->setValue(0);
}

void GsplatDisplay::onSplatPathChanged()
{
  if (!scene_manager_ || !scene_node_ || !splat_cloud_) {
    return;
  }

  splat_cloud_->clear();

  const QString path = splat_path_property_->getString();
  if (path.isEmpty()) {
    setStatus(
      rviz_common::properties::StatusProperty::Warn,
      "Splat File", "No file selected.");
    return;
  }

  std::string error_msg;
  int sh_degree = 0;
  std::vector<SplatGPU> gaussians = loadPly(path.toStdString(), error_msg, sh_degree);

  if (!error_msg.empty()) {
    setStatus(
      rviz_common::properties::StatusProperty::Error,
      "Splat File", QString::fromStdString(error_msg));
    return;
  }

  const int count = static_cast<int>(gaussians.size());
  splat_cloud_->setSplats(std::move(gaussians), sh_degree);

  sh_degree_property_->setMax(sh_degree);
  sh_degree_property_->setValue(std::min(1, sh_degree));

  setStatus(
    rviz_common::properties::StatusProperty::Ok,
    "Splat File",
    QString("Loaded %1 gaussians").arg(count));
}

void GsplatDisplay::onShDegreeChanged()
{
  if (splat_cloud_) {
    splat_cloud_->setShDegree(sh_degree_property_->getInt());
  }
}

void GsplatDisplay::onTopicChanged()
{
  unsubscribe();
  if (isEnabled()) {
    subscribe();
  }
}

void GsplatDisplay::subscribe()
{
  if (!context_ || subscription_) {
    return;
  }

  const std::string topic = topic_property_->getTopicStd();
  if (topic.empty()) {
    setStatus(
      rviz_common::properties::StatusProperty::Warn,
      "Topic", "No topic selected.");
    return;
  }

  auto ros_node = context_->getRosNodeAbstraction().lock();
  if (!ros_node) {
    setStatus(
      rviz_common::properties::StatusProperty::Error,
      "Topic", "RViz ROS node unavailable.");
    return;
  }

  // TRANSIENT_LOCAL matches the latched publisher so we receive the last
  // message on late-subscribe; the executor runs in rviz_common.
  rclcpp::QoS qos(1);
  qos.reliable().transient_local();

  try {
    subscription_ = ros_node->get_raw_node()->create_subscription<splat_msgs::msg::SplatArray>(
      topic, qos,
      [this](splat_msgs::msg::SplatArray::ConstSharedPtr msg) {
        this->splatArrayCallback(std::move(msg));
      });
  } catch (const std::exception & e) {
    setStatus(
      rviz_common::properties::StatusProperty::Error,
      "Topic", QString("Subscription failed: %1").arg(e.what()));
    return;
  }

  setStatus(
    rviz_common::properties::StatusProperty::Ok,
    "Topic",
    QString("Subscribed to %1").arg(QString::fromStdString(topic)));
}

void GsplatDisplay::unsubscribe()
{
  subscription_.reset();
}

void GsplatDisplay::splatArrayCallback(splat_msgs::msg::SplatArray::ConstSharedPtr msg)
{
  // Store only the latest message; apply on the main thread in update().
  std::lock_guard<std::mutex> lock(pending_msg_mutex_);
  pending_msg_ = std::move(msg);
}

void GsplatDisplay::applyPendingMessage()
{
  splat_msgs::msg::SplatArray::ConstSharedPtr msg;
  {
    std::lock_guard<std::mutex> lock(pending_msg_mutex_);
    msg.swap(pending_msg_);
  }
  if (!msg || !splat_cloud_) {
    return;
  }

  int sh_degree = 0;
  std::vector<SplatGPU> gaussians = splatsFromMessage(*msg, sh_degree);
  const int count = static_cast<int>(gaussians.size());

  splat_cloud_->setSplats(std::move(gaussians), sh_degree);

  sh_degree_property_->setMax(sh_degree);
  const int current = sh_degree_property_->getInt();
  if (current > sh_degree) {
    sh_degree_property_->setValue(sh_degree);
  }

  setStatus(
    rviz_common::properties::StatusProperty::Ok,
    "Topic",
    QString("Received %1 gaussians (SH degree %2)").arg(count).arg(sh_degree));
}

}  // namespace displays
}  // namespace gsplat_rviz_trials

PLUGINLIB_EXPORT_CLASS(
  gsplat_rviz_trials::displays::GsplatDisplay,
  rviz_common::Display)
