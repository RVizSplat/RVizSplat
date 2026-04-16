#ifndef GSPLAT_RVIZ_TRIALS__PLY_LOADER_HPP_
#define GSPLAT_RVIZ_TRIALS__PLY_LOADER_HPP_

#include <string>
#include <vector>

#include <OgreColourValue.h>
#include <OgreVector3.h>

#include "gsplat_rviz_trials/visibility_control.hpp"

namespace gsplat_rviz_trials
{

/**
 * @brief Data for one Gaussian splat, ready to pass to the Splat constructor.
 *
 * All values are already converted from the raw PLY representation:
 *  - position : world-space centre
 *  - color    : RGBA in [0,1]; RGB from SH DC coefficients, A from sigmoid(opacity)
 *  - covariance: upper-triangular 3D covariance {v11,v12,v13,v22,v23,v33}
 *                computed as  R * diag(s²) * Rᵀ  from the stored scale+quaternion
 */
struct GaussianData
{
  Ogre::Vector3 position;
  Ogre::ColourValue color;
  float covariance[6];  // v11, v12, v13, v22, v23, v33

  // Spherical harmonics coefficients in coefficient-major order.
  // sh[3*i .. 3*i+2] = (R, G, B) for SH basis function i.
  // Index 0 is the DC term (f_dc_0/1/2); indices 1..15 come from f_rest_*.
  // Unused slots (when sh_degree < 3) are zero-filled.
  float sh[48];   // 16 basis functions × 3 channels
  int sh_degree;  // 0, 1, 2, or 3
};

/**
 * @brief Load all Gaussian splats from a 3DGS-format PLY file.
 *
 * Supports binary_little_endian and ascii PLY formats.
 * On failure returns an empty vector and fills error_msg.
 */
GSPLAT_RVIZ_TRIALS_PUBLIC
std::vector<GaussianData> loadPly(
  const std::string & path,
  std::string & error_msg);

}  // namespace gsplat_rviz_trials

#endif  // GSPLAT_RVIZ_TRIALS__PLY_LOADER_HPP_
