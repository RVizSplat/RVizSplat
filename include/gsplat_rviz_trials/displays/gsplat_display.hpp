#ifndef GSPLAT_RVIZ_TRIALS__DISPLAYS__GSPLAT_DISPLAY_HPP_
#define GSPLAT_RVIZ_TRIALS__DISPLAYS__GSPLAT_DISPLAY_HPP_

#include <memory>

#include "rviz_common/display.hpp"
#include "rviz_common/properties/file_picker_property.hpp"
#include "gsplat_rviz_trials/visibility_control.hpp"

namespace gsplat_rviz_trials
{
class Splat;

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

private:
  rviz_common::properties::FilePickerProperty * splat_path_property_;
  std::unique_ptr<Splat> splat_;
};

}  // namespace displays
}  // namespace gsplat_rviz_trials

#endif  // GSPLAT_RVIZ_TRIALS__DISPLAYS__GSPLAT_DISPLAY_HPP_
