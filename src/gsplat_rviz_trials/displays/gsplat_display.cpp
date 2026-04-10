#include "gsplat_rviz_trials/displays/gsplat_display.hpp"

#include <QFileDialog>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>
#include <OgreMaterialManager.h>
#include <OgreCamera.h>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/view_manager.hpp"
#include "rviz_common/view_controller.hpp"
#include "rviz_common/properties/status_property.hpp"
#include "gsplat_rviz_trials/splat.hpp"

namespace gsplat_rviz_trials
{
namespace displays
{

GsplatDisplay::GsplatDisplay()
{
  splat_path_property_ = new rviz_common::properties::FilePickerProperty(
    "Splat File", "",
    "Path to the Gaussian Splat file to visualize.",
    this, SLOT(onSplatPathChanged()),
    this,
    QFileDialog::ExistingFile);
}

GsplatDisplay::~GsplatDisplay() = default;

void GsplatDisplay::onInitialize()
{
  rviz_common::Display::onInitialize();
  splat_ = std::make_unique<gsplat_rviz_trials::Splat>(scene_manager_, scene_node_);
}

void GsplatDisplay::onEnable()
{
}

void GsplatDisplay::onDisable()
{
}

void GsplatDisplay::update(
  std::chrono::nanoseconds /*wall_dt*/,
  std::chrono::nanoseconds /*ros_dt*/)
{
  if (splat_ && context_ && context_->getViewManager()) {
    rviz_common::ViewController * view_controller = context_->getViewManager()->getCurrent();
    if (view_controller) {
      splat_->update(view_controller->getCamera());
    }
  }
}

void GsplatDisplay::reset()
{
  rviz_common::Display::reset();
}

void GsplatDisplay::onSplatPathChanged()
{
  const QString path = splat_path_property_->getString();

  if (path.isEmpty()) {
    setStatus(
      rviz_common::properties::StatusProperty::Warn,
      "Splat File",
      "No file selected.");
    return;
  }

  setStatus(
    rviz_common::properties::StatusProperty::Ok,
    "Splat File",
    path);

  // TODO: load the splat file at path.toStdString()
}

}  // namespace displays
}  // namespace gsplat_rviz_trials

PLUGINLIB_EXPORT_CLASS(
  gsplat_rviz_trials::displays::GsplatDisplay,
  rviz_common::Display)
