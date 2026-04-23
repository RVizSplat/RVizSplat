#ifndef GSPLAT_RVIZ_TRIALS__DISPLAYS__GSPLAT_DISPLAY_HPP_
#define GSPLAT_RVIZ_TRIALS__DISPLAYS__GSPLAT_DISPLAY_HPP_

#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <splat_msgs/msg/splat_array.hpp>

#include "rviz_common/display.hpp"
#include "rviz_common/properties/file_picker_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"
#include "gsplat_rviz_trials/visibility_control.hpp"

namespace gsplat_rviz_trials
{
class SplatCloud;

namespace displays
{

class GSPLAT_RVIZ_TRIALS_PUBLIC GsplatDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  GsplatDisplay();
  ~GsplatDisplay() override;

protected:
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void update(std::chrono::nanoseconds wall_dt, std::chrono::nanoseconds ros_dt) override;
  void reset() override;

private Q_SLOTS:
  void onSplatPathChanged();
  void onShDegreeChanged();
  void onTopicChanged();

private:
  void subscribe();
  void unsubscribe();
  void splatArrayCallback(splat_msgs::msg::SplatArray::ConstSharedPtr msg);
  void applyPendingMessage();

  rviz_common::properties::FilePickerProperty * splat_path_property_;
  rviz_common::properties::RosTopicProperty *   topic_property_;
  rviz_common::properties::IntProperty *        sh_degree_property_;

  std::unique_ptr<SplatCloud> splat_cloud_;

  // pending_msg_mutex_/pending_msg_ are touched by the subscription callback,
  // so they must outlive subscription_. Declaration order matters: members
  // are destroyed in reverse declaration order, so subscription_ is declared
  // last to guarantee it (and any in-flight callback) is torn down first.
  std::mutex pending_msg_mutex_;
  splat_msgs::msg::SplatArray::ConstSharedPtr pending_msg_;
  rclcpp::Subscription<splat_msgs::msg::SplatArray>::SharedPtr subscription_;
};

}  // namespace displays
}  // namespace gsplat_rviz_trials

#endif  // GSPLAT_RVIZ_TRIALS__DISPLAYS__GSPLAT_DISPLAY_HPP_
