#ifndef GSPLAT_RVIZ_TRIALS__DISPLAYS__GSPLAT_DISPLAY_HPP_
#define GSPLAT_RVIZ_TRIALS__DISPLAYS__GSPLAT_DISPLAY_HPP_

#include <cstdint>
#include <memory>
#include <vector>

#include <OgreVector3.h>

#include "rviz_common/display.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/file_picker_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"
#include "gsplat_rviz_trials/splat_loaders/i_splat_source.hpp"
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
  void reset() override;

private Q_SLOTS:
  void onSourceModeChanged();
  void onSplatPathChanged();
  void onShDegreeChanged();
  void onAlphaThresholdChanged();
  void onTopicChanged();
  void onSorterKindChanged();

private:
  enum class SourceKind { None, File, Topic };
  enum class SourceMode { File = 0, Topic = 1 };

  void rebuildSorter();
  void installSource(std::unique_ptr<ISplatSource> source, SourceKind kind);
  void onLoadResult(LoadResult result, SourceKind kind, uint64_t gen);
  SourceMode currentMode() const;

  rviz_common::properties::EnumProperty *       source_mode_property_;
  rviz_common::properties::FilePickerProperty * splat_path_property_;
  rviz_common::properties::RosTopicProperty *   topic_property_;
  rviz_common::properties::IntProperty *        sh_degree_property_;
  rviz_common::properties::FloatProperty *      alpha_threshold_property_;
  rviz_common::properties::EnumProperty *       sorter_kind_property_;

  std::unique_ptr<SplatCloud>   splat_cloud_;
  std::unique_ptr<ISplatSource> source_;
  SourceKind                    source_kind_ = SourceKind::None;
  // Bumped each time source_ is (re)assigned or cleared. Queued main-thread
  // deliveries from a prior source compare the captured generation against
  // the current one and drop if stale — prevents an in-flight callback from
  // clobbering state after the user switches topic/file.
  uint64_t                      source_gen_  = 0;
};

}  // namespace displays
}  // namespace gsplat_rviz_trials

#endif  // GSPLAT_RVIZ_TRIALS__DISPLAYS__GSPLAT_DISPLAY_HPP_
