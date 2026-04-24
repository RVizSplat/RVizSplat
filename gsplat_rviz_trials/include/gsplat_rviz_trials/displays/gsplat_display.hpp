#ifndef GSPLAT_RVIZ_TRIALS__DISPLAYS__GSPLAT_DISPLAY_HPP_
#define GSPLAT_RVIZ_TRIALS__DISPLAYS__GSPLAT_DISPLAY_HPP_

#include <memory>
#include <vector>

#include <OgreVector3.h>

#include "rviz_common/display.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/file_picker_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "gsplat_rviz_trials/visibility_control.hpp"

namespace Ogre
{
class Viewport;
}

namespace gsplat_rviz_trials
{
class ISplatSorter;
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
  void onTopicChanged();
  void onSorterKindChanged();
  void onClipChanged();
  void onTransparencyModeChanged();
  void onWboitTuningChanged();

private:
  void rebuildSorter();
  void pollSource();
  void applyTransparencyMode();

  rviz_common::properties::FilePickerProperty * splat_path_property_;
  rviz_common::properties::RosTopicProperty *   topic_property_;
  rviz_common::properties::IntProperty *        sh_degree_property_;
  rviz_common::properties::EnumProperty *       sorter_kind_property_;

  // ROI clip — AABB in the scene's local frame.
  rviz_common::properties::BoolProperty *       clip_enabled_property_;
  rviz_common::properties::VectorProperty *     clip_min_property_;
  rviz_common::properties::VectorProperty *     clip_max_property_;

  // Advanced — transparency fallback. Default is Sorted; WBOIT lives under
  // an "Advanced" group because Sorted is the right answer for most users.
  rviz_common::properties::EnumProperty *       transparency_mode_property_;
  rviz_common::properties::FloatProperty *      wboit_weight_scale_property_;
  rviz_common::properties::FloatProperty *      wboit_weight_exponent_property_;
  rviz_common::properties::FloatProperty *      wboit_alpha_discard_property_;

  std::unique_ptr<SplatCloud>   splat_cloud_;
  std::unique_ptr<ISplatSorter> sorter_;
  std::unique_ptr<ISplatSource> source_;

  // Cached centres so the sorter can be rebuilt without re-loading data.
  std::vector<Ogre::Vector3> centers_cache_;

  // Cached viewport pointer for compositor attach/detach across mode switches.
  Ogre::Viewport * compositor_viewport_{nullptr};
  bool             wboit_compositor_active_{false};
};

}  // namespace displays
}  // namespace gsplat_rviz_trials

#endif  // GSPLAT_RVIZ_TRIALS__DISPLAYS__GSPLAT_DISPLAY_HPP_
