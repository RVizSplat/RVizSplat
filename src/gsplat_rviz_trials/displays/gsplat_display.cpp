#include "gsplat_rviz_trials/displays/gsplat_display.hpp"

#include <QFileDialog>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/status_property.hpp"

#include "gsplat_rviz_trials/ply_loader.hpp"
#include "gsplat_rviz_trials/splat_cloud.hpp"

namespace gsplat_rviz_trials
{
namespace displays
{

GsplatDisplay::GsplatDisplay()
{
  splat_path_property_ = new rviz_common::properties::FilePickerProperty(
    "Splat File", "",
    "Path to a 3DGS-format PLY file to visualize.",
    this, SLOT(onSplatPathChanged()),
    this,
    QFileDialog::ExistingFile);
}

GsplatDisplay::~GsplatDisplay() = default;

void GsplatDisplay::onInitialize()
{
  rviz_common::Display::onInitialize();
  splat_cloud_ = std::make_unique<SplatCloud>(scene_node_);
}

void GsplatDisplay::onEnable() {}
void GsplatDisplay::onDisable() {}

void GsplatDisplay::update(
  std::chrono::nanoseconds /*wall_dt*/,
  std::chrono::nanoseconds /*ros_dt*/) {}

void GsplatDisplay::reset()
{
  rviz_common::Display::reset();
  if (splat_cloud_) {
    splat_cloud_->clear();
  }
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
  std::vector<GaussianData> gaussians = loadPly(path.toStdString(), error_msg);

  if (!error_msg.empty()) {
    setStatus(
      rviz_common::properties::StatusProperty::Error,
      "Splat File", QString::fromStdString(error_msg));
    return;
  }

  for (const GaussianData & g : gaussians) {
    splat_cloud_->addSplat(g.position, g.covariance, g.color, g.sh, g.sh_degree);
  }
  scene_node_->needUpdate();

  setStatus(
    rviz_common::properties::StatusProperty::Ok,
    "Splat File",
    QString("Loaded %1 gaussians").arg(static_cast<int>(gaussians.size())));
}

}  // namespace displays
}  // namespace gsplat_rviz_trials

PLUGINLIB_EXPORT_CLASS(
  gsplat_rviz_trials::displays::GsplatDisplay,
  rviz_common::Display)
