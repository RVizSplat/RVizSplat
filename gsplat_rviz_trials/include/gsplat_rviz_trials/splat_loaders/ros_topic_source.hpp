#ifndef GSPLAT_RVIZ_TRIALS__ROS_TOPIC_SOURCE_HPP_
#define GSPLAT_RVIZ_TRIALS__ROS_TOPIC_SOURCE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <splat_msgs/msg/splat_array.hpp>

#include "gsplat_rviz_trials/splat_loaders/i_splat_source.hpp"

namespace gsplat_rviz_trials
{

// Subscribes to a splat_msgs/SplatArray topic. The callback stashes the most
// recent message; poll() drains it (wire-format → SplatGPU conversion) and
// returns a LoadResult. Older messages are discarded if the main thread
// doesn't keep up — visualization always reflects the latest frame.
class RosTopicSource : public ISplatSource
{
public:
  RosTopicSource(
    rclcpp::Node::SharedPtr node,
    const std::string & topic);
  ~RosTopicSource() override;

  std::unique_ptr<LoadResult> poll() override;

  // Non-empty on subscription setup failure; use before poll() to surface the
  // error via status without waiting for a message.
  const std::string & initError() const { return init_error_; }

private:
  void callback(splat_msgs::msg::SplatArray::ConstSharedPtr msg);

  std::string init_error_;
  std::mutex mutex_;
  splat_msgs::msg::SplatArray::ConstSharedPtr pending_msg_;
  // subscription_ is declared last so it (and any in-flight callback) is torn
  // down before mutex_/pending_msg_ during destruction.
  rclcpp::Subscription<splat_msgs::msg::SplatArray>::SharedPtr subscription_;
};

}  // namespace gsplat_rviz_trials

#endif  // GSPLAT_RVIZ_TRIALS__ROS_TOPIC_SOURCE_HPP_
