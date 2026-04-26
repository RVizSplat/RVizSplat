#ifndef GSPLAT_RVIZ_PLUGIN__ROS_TOPIC_SOURCE_HPP_
#define GSPLAT_RVIZ_PLUGIN__ROS_TOPIC_SOURCE_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <gsplat_msgs/msg/splat_array.hpp>

#include "gsplat_rviz_plugin/splat_loaders/i_splat_source.hpp"

namespace gsplat_rviz_plugin
{

class RosTopicSource : public ISplatSource
{
public:
  RosTopicSource(
    rclcpp::Node::SharedPtr node,
    const std::string & topic);
  ~RosTopicSource() override;

  void start(Callback cb) override;

private:
  rclcpp::Node::SharedPtr node_;
  std::string topic_;
  Callback callback_;
  rclcpp::Subscription<gsplat_msgs::msg::SplatArray>::SharedPtr subscription_;
};

}  // namespace gsplat_rviz_plugin

#endif  // GSPLAT_RVIZ_PLUGIN__ROS_TOPIC_SOURCE_HPP_
