#include "gsplat_rviz_trials/splat_loaders/ros_topic_source.hpp"

#include <utility>

namespace gsplat_rviz_trials
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
LoadResult splatsFromMessage(const splat_msgs::msg::SplatArray & msg)
{
  LoadResult out;
  out.sh_degree = msg.splats.empty()
    ? 0
    : static_cast<int>(msg.splats.front().harmonics_deg);
  const int num_coefs = (out.sh_degree + 1) * (out.sh_degree + 1);
  out.splats.reserve(msg.splats.size());

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

    out.splats.push_back(g);
  }
  return out;
}

}  // namespace

RosTopicSource::RosTopicSource(
  rclcpp::Node::SharedPtr node,
  const std::string & topic)
{
  if (!node) {
    init_error_ = "ROS node unavailable.";
    return;
  }
  if (topic.empty()) {
    init_error_ = "No topic selected.";
    return;
  }

  // TRANSIENT_LOCAL matches the latched publisher so late subscribers receive
  // the most recent message immediately.
  rclcpp::QoS qos(1);
  qos.reliable().transient_local();

  try {
    subscription_ = node->create_subscription<splat_msgs::msg::SplatArray>(
      topic, qos,
      [this](splat_msgs::msg::SplatArray::ConstSharedPtr msg) {
        this->callback(std::move(msg));
      });
  } catch (const std::exception & e) {
    init_error_ = std::string("Subscription failed: ") + e.what();
  }
}

RosTopicSource::~RosTopicSource()
{
  subscription_.reset();
}

void RosTopicSource::callback(splat_msgs::msg::SplatArray::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  pending_msg_ = std::move(msg);
}

std::unique_ptr<LoadResult> RosTopicSource::poll()
{
  splat_msgs::msg::SplatArray::ConstSharedPtr msg;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    msg.swap(pending_msg_);
  }
  if (!msg) return nullptr;

  auto r = std::make_unique<LoadResult>();
  *r = splatsFromMessage(*msg);
  return r;
}

}  // namespace gsplat_rviz_trials
