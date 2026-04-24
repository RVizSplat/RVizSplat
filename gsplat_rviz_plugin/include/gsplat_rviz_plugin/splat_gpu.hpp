#ifndef GSPLAT_RVIZ_PLUGIN__SPLAT_GPU_HPP_
#define GSPLAT_RVIZ_PLUGIN__SPLAT_GPU_HPP_

#include <cstddef>
#include <cstdint>

namespace gsplat_rviz_plugin
{

// GPU representation of one Gaussian splat packed for a GL_RGBA32F TBO.
// Each splat occupies exactly 19 vec4 texels (304 bytes).
//
//  Byte offset │ Field      │ TBO texel (base + N)
//  ────────────┼────────────┼─────────────────────
//    0         │ center[3]  │ base+0  .xyz
//   12         │ alpha      │ base+0  .w
//   16         │ covA[3]    │ base+1  .xyz   {v11, v12, v13}
//   28         │ _pad0      │ base+1  .w
//   32         │ covB[3]    │ base+2  .xyz   {v22, v23, v33}
//   44         │ _pad1      │ base+2  .w
//   48         │ sh[16][4]  │ base+3 … base+18  (RGB per coeff, w=0)
//  304         │ end        │
struct alignas(4) SplatGPU
{
  float center[3];
  float alpha;
  float covA[3];   // {v11, v12, v13}
  float _pad0;
  float covB[3];   // {v22, v23, v33}
  float _pad1;
  float sh[16][4]; // sh[i][0..2] = RGB, sh[i][3] = 0
};

static_assert(sizeof(SplatGPU) == 304, "SplatGPU size mismatch");
static_assert(offsetof(SplatGPU, covA) == 16, "covA offset");
static_assert(offsetof(SplatGPU, covB) == 32, "covB offset");
static_assert(offsetof(SplatGPU, sh)   == 48, "sh offset");

}  // namespace gsplat_rviz_plugin

#endif  // GSPLAT_RVIZ_PLUGIN__SPLAT_GPU_HPP_
