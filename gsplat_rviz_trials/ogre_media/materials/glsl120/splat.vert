#version 330 core
#extension GL_ARB_shading_language_packing : require

// Source 0: quad corner (±2, ±2) — per-vertex
layout(location = 0) in vec2 vertex;
// Source 1: sorted splat index — per-instance (divisor = 1)
layout(location = 8) in float uv0;

// Compact base TBO: 2 uvec4 texels per splat (32 B).
//   texel 0.xyz = uintBitsToFloat → center
//   texel 0.w   = packHalf2x16(cov00, cov01)
//   texel 1.x   = packHalf2x16(cov02, cov11)
//   texel 1.y   = packHalf2x16(cov12, cov22)
//   texel 1.z   = packUnorm4x8(r, g, b, a)   ← DC pre-baked to uint8
//   texel 1.w   = reserved
uniform usamplerBuffer u_splats;

// Optional SH TBO: (sh_degree+1)² - 1 RGBA16F texels per splat, non-DC coeffs.
// Bound only when sh_degree > 0; shader avoids sampling it otherwise.
uniform samplerBuffer u_sh;

uniform mat4          view_matrix;
uniform mat4          projectionMatrix;
uniform vec3          cam_pos_world;      // camera position in world space
uniform int           sh_degree;

// ROI clip box (axis-aligned, in the scene_node_'s local frame which is the
// Reference Frame's coordinates on the host side).
uniform int           u_clip_enabled;
uniform vec3          u_clip_min;
uniform vec3          u_clip_max;

// Splat-tight forward-z warp bounds used by the WBOIT weight function.
// Far = -1 signals "unset" → fragment shaders fall back to gl_FragCoord.z.
uniform float         u_splat_z_near;
uniform float         u_splat_z_far;

out vec4  vColor;
out vec2  vPosition;
out float v_splat_z_warped;     // splat-center view-z warped into [0,1]

// ── SH constants (Y_l^m coefficients) ──────────────────────────────────────
const float SH_C1    =  0.4886025119029199;
const float SH_C2_0  =  1.0925484305920792;
const float SH_C2_1  = -1.0925484305920792;
const float SH_C2_2  =  0.31539156525252005;
const float SH_C2_3  = -1.0925484305920792;
const float SH_C2_4  =  0.5462742152960396;
const float SH_C3_0  = -0.5900435899266435;
const float SH_C3_1  =  2.890611442640554;
const float SH_C3_2  = -0.4570457994644658;
const float SH_C3_3  =  0.3731763325901154;
const float SH_C3_4  = -0.4570457994644658;
const float SH_C3_5  =  1.445305721320277;
const float SH_C3_6  = -0.5900435899266435;

// Number of non-DC SH coefficients for a given degree.
int shCoeffsNonDC(int d) { return (d + 1) * (d + 1) - 1; }

vec3 fetchSH(int base, int k) { return texelFetch(u_sh, base + k).rgb; }

// Evaluate SH orders 1..sh_degree at direction d, returning RGB increment
// to add to the DC base colour.
vec3 evalSHOrders1Plus(int splat_id, vec3 d)
{
    int base = splat_id * shCoeffsNonDC(sh_degree);
    vec3 rgb = vec3(0.0);

    // Order 1 (3 coeffs)
    rgb += -SH_C1 * d.y * fetchSH(base, 0)
         +  SH_C1 * d.z * fetchSH(base, 1)
         + -SH_C1 * d.x * fetchSH(base, 2);

    if (sh_degree >= 2) {
        float xx = d.x * d.x, yy = d.y * d.y, zz = d.z * d.z;
        float xy = d.x * d.y, yz = d.y * d.z, xz = d.x * d.z;
        rgb += SH_C2_0 * xy               * fetchSH(base, 3)
             + SH_C2_1 * yz               * fetchSH(base, 4)
             + SH_C2_2 * (2.0*zz-xx-yy)   * fetchSH(base, 5)
             + SH_C2_3 * xz               * fetchSH(base, 6)
             + SH_C2_4 * (xx - yy)        * fetchSH(base, 7);

        if (sh_degree >= 3) {
            rgb += SH_C3_0 * d.y * (3.0*xx - yy)              * fetchSH(base, 8)
                 + SH_C3_1 * d.z * xy                          * fetchSH(base, 9)
                 + SH_C3_2 * d.y * (4.0*zz - xx - yy)          * fetchSH(base, 10)
                 + SH_C3_3 * d.z * (2.0*zz - 3.0*xx - 3.0*yy)  * fetchSH(base, 11)
                 + SH_C3_4 * d.x * (4.0*zz - xx - yy)          * fetchSH(base, 12)
                 + SH_C3_5 * d.z * (xx - yy)                   * fetchSH(base, 13)
                 + SH_C3_6 * d.x * (xx - 3.0*yy)               * fetchSH(base, 14);
        }
    }
    return rgb;
}

void main()
{
    int splat_id = int(uv0);
    int base     = splat_id * 2;

    uvec4 t0 = texelFetch(u_splats, base + 0);
    uvec4 t1 = texelFetch(u_splats, base + 1);

    vec3 center = uintBitsToFloat(t0.xyz);

    // ROI clip: reject splats whose centre is outside [Clip Min, Clip Max].
    // z = 2 places the quad past the far plane → rasteriser discards.
    if (u_clip_enabled != 0) {
        if (any(lessThan(center, u_clip_min)) ||
            any(greaterThan(center, u_clip_max))) {
            gl_Position = vec4(0.0, 0.0, 2.0, 1.0);
            return;
        }
    }

    vec2 c0001 = unpackHalf2x16(t0.w);
    vec2 c0211 = unpackHalf2x16(t1.x);
    vec2 c1222 = unpackHalf2x16(t1.y);

    float c00 = c0001.x, c01 = c0001.y;
    float c02 = c0211.x, c11 = c0211.y;
    float c12 = c1222.x, c22 = c1222.y;

    vec4 rgba = unpackUnorm4x8(t1.z);
    if (rgba.a < 0.4) {
        gl_Position = vec4(0.0, 0.0, 2.0, 1.0);
        return;
    }

    // ── Project center ────────────────────────────────────────────────────────
    vec4 camspace = view_matrix * vec4(center, 1.0);
    vec4 pos2d    = projectionMatrix * camspace;

    float bounds = 1.2 * pos2d.w;
    if (pos2d.z < -pos2d.w
        || pos2d.x < -bounds || pos2d.x > bounds
        || pos2d.y < -bounds || pos2d.y > bounds) {
        gl_Position = vec4(0.0, 0.0, 2.0, 1.0);
        return;
    }

    // ── EWA: 3D covariance → 2D screen covariance ─────────────────────────────
    mat3 J = mat3(
        -projectionMatrix[0][0] / camspace.z, 0.0,
        (projectionMatrix[0][0] * camspace.x) / (camspace.z * camspace.z),
        0.0, -projectionMatrix[1][1] / camspace.z,
        (projectionMatrix[1][1] * camspace.y) / (camspace.z * camspace.z),
        0.0, 0.0, 0.0
    );

    mat3 cov3d = mat3(
        c00, c01, c02,
        c01, c11, c12,
        c02, c12, c22
    );

    mat3 R   = transpose(mat3(view_matrix));
    mat3 T   = R * J;
    mat3 cov = transpose(T) * cov3d * T;

    vec3 vCenter = vec3(pos2d) / pos2d.w;

    float d1 = cov[0][0], od = cov[0][1], d2 = cov[1][1];
    float mid     = 0.5 * (d1 + d2);
    float radius  = length(vec2((d1 - d2) * 0.5, od));
    float lambda1 = mid + radius;
    float lambda2 = mid - radius;
    vec2 diag = normalize(vec2(od, lambda1 - d1));
    vec2 v1   = sqrt(max(2. * lambda1, 0.0)) * diag;
    vec2 v2   = sqrt(max(2. * lambda2, 0.0)) * vec2(diag.y, -diag.x);

    // Clamp projected eigen-axes.  The EWA Jacobian grows as 1/z_cam², so a
    // splat near the near plane can rasterise a quad covering a large fraction
    // of the framebuffer and tank framerate on close zoom.  Cap at 0.75 NDC
    // half-extent (~720 px at 1080p), matching antimatter15/splat's 1024-px cap
    // at 2K.
    const float kMaxAxisNDC = 0.75;
    float m1 = length(v1);
    if (m1 > kMaxAxisNDC) v1 *= kMaxAxisNDC / m1;
    float m2 = length(v2);
    if (m2 > kMaxAxisNDC) v2 *= kMaxAxisNDC / m2;

    // ── Colour: uint8 DC base, plus optional SH1..3 contribution ─────────────
    // Per-splat view direction (3DGS / Inria convention).  Using a scene-wide
    // `view_direction` auto-param would collapse SH to a constant argument and
    // give the wrong colour at sh_degree > 0 for every splat not on-axis.
    vec3 rgb = rgba.rgb;
    if (sh_degree > 0) {
        vec3 dir = normalize(center - cam_pos_world);
        rgb = clamp(rgb + evalSHOrders1Plus(splat_id, dir), 0.0, 1.0);
    }
    vColor    = vec4(rgb, rgba.a);
    vPosition = vertex;

    // Splat-tight warped depth in [0,1]; sentinel -1 if bounds unset.  Ogre
    // view space looks down -Z, so forward distance is -camspace.z.  All four
    // quad vertices share this value, so it interpolates to a constant per
    // splat — exactly the per-splat depth WBOIT's weight function wants.
    float z_span = u_splat_z_far - u_splat_z_near;
    v_splat_z_warped = (z_span > 1e-3)
        ? clamp((-camspace.z - u_splat_z_near) / z_span, 0.0, 1.0)
        : -1.0;

    gl_Position = vec4(
        vCenter.xy + vertex.x * v1 + vertex.y * v2,
        vCenter.z, 1.0);
}
