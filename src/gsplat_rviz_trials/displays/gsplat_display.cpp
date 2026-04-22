#include "gsplat_rviz_trials/displays/gsplat_display.hpp"

#include <QFileDialog>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/status_property.hpp"

#include "gsplat_rviz_trials/ply_loader.hpp"
#include "gsplat_rviz_trials/splat_cloud.hpp"
#include "gsplat_rviz_trials/splat_gpu.hpp"

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

  sh_degree_property_ = new rviz_common::properties::IntProperty(
    "SH Degree", 0,
    "Spherical harmonics degree used for view-dependent colour (0 = DC only). "
    "Lower values are faster; maximum is set by the loaded PLY file.",
    this, SLOT(onShDegreeChanged()), this);
  sh_degree_property_->setMin(0);
  sh_degree_property_->setMax(0);
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
  sh_degree_property_->setMax(0);
  sh_degree_property_->setValue(0);
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
  int sh_degree = 0;
  std::vector<SplatGPU> gaussians = loadPly(path.toStdString(), error_msg, sh_degree);

  if (!error_msg.empty()) {
    setStatus(
      rviz_common::properties::StatusProperty::Error,
      "Splat File", QString::fromStdString(error_msg));
    return;
  }

  const int count = static_cast<int>(gaussians.size());
  splat_cloud_->setSplats(std::move(gaussians), sh_degree);

  // lean-mode Phase 1: only DC (SH0) is rendered; higher orders will come back
  // once the SH TBO is wired up in Phase 2.
  (void)sh_degree;
  sh_degree_property_->setMax(0);
  sh_degree_property_->setValue(0);

  setStatus(
    rviz_common::properties::StatusProperty::Ok,
    "Splat File",
    QString("Loaded %1 gaussians").arg(count));
}

void GsplatDisplay::onShDegreeChanged()
{
  if (splat_cloud_) {
    splat_cloud_->setShDegree(sh_degree_property_->getInt());
  }
}

}  // namespace displays
}  // namespace gsplat_rviz_trials

PLUGINLIB_EXPORT_CLASS(
  gsplat_rviz_trials::displays::GsplatDisplay,
  rviz_common::Display)
