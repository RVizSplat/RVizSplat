#include "gsplat_rviz_trials/displays/gsplat_display.hpp"

#include <QFileDialog>

#include <OgreCamera.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/status_property.hpp"
#include "rviz_common/view_controller.hpp"
#include "rviz_common/view_manager.hpp"

#include "gsplat_rviz_trials/ply_loader.hpp"
#include "gsplat_rviz_trials/splat.hpp"

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
  // Actual splats are created in onSplatPathChanged() when a file is selected.
}

void GsplatDisplay::onEnable() {}
void GsplatDisplay::onDisable() {}

void GsplatDisplay::update(
  std::chrono::nanoseconds /*wall_dt*/,
  std::chrono::nanoseconds /*ros_dt*/)
{
  if (!context_ || !context_->getViewManager()) return;
  rviz_common::ViewController * vc = context_->getViewManager()->getCurrent();
  if (!vc) return;
  Ogre::Camera * cam = vc->getCamera();
  for (auto & splat : splats_) {
    splat->update(cam);
  }
}

void GsplatDisplay::reset()
{
  rviz_common::Display::reset();
  splats_.clear();
}

void GsplatDisplay::onSplatPathChanged()
{
  // Guard: scene resources not yet ready before onInitialize().
  if (!scene_manager_ || !scene_node_) return;

  const QString path = splat_path_property_->getString();
  if (path.isEmpty()) {
    setStatus(
      rviz_common::properties::StatusProperty::Warn,
      "Splat File", "No file selected.");
    splats_.clear();
    return;
  }

  // Destroy previous scene objects before loading the new file.
  splats_.clear();

  std::string error_msg;
  std::vector<GaussianData> gaussians = loadPly(path.toStdString(), error_msg);

  if (!error_msg.empty()) {
    setStatus(
      rviz_common::properties::StatusProperty::Error,
      "Splat File", QString::fromStdString(error_msg));
    return;
  }

  // Create one Splat per gaussian; each clones its own material so GPU
  // parameters are fully independent (see Splat constructor).
  splats_.reserve(gaussians.size());
  for (const GaussianData & g : gaussians) {
    splats_.push_back(std::make_unique<Splat>(
      scene_manager_, scene_node_,
      g.position, g.covariance, g.color,
      g.sh, g.sh_degree));
  }

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
