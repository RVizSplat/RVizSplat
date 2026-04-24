#ifndef GSPLAT_RVIZ_TRIALS__DISPLAYS__GSPLAT_DISPLAY_HPP_
#define GSPLAT_RVIZ_TRIALS__DISPLAYS__GSPLAT_DISPLAY_HPP_

#include <memory>
#include <vector>

#include <OgreVector3.h>

#include "rviz_common/display.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/file_picker_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"
#include "gsplat_rviz_trials/visibility_control.hpp"

namespace gsplat_rviz_trials
{
class ISplatSource;
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
  void onAlphaThresholdChanged();
  void onTopicChanged();
  void onSorterKindChanged();

private:
  void rebuildSorter();
  void pollSource();

  rviz_common::properties::FilePickerProperty * splat_path_property_;
  rviz_common::properties::RosTopicProperty *   topic_property_;
  rviz_common::properties::IntProperty *        sh_degree_property_;
  rviz_common::properties::FloatProperty *      alpha_threshold_property_;
  rviz_common::properties::EnumProperty *       sorter_kind_property_;

  std::unique_ptr<SplatCloud>   splat_cloud_;
  std::unique_ptr<ISplatSource> source_;
};

}  // namespace displays
}  // namespace gsplat_rviz_trials

#endif  // GSPLAT_RVIZ_TRIALS__DISPLAYS__GSPLAT_DISPLAY_HPP_
