#version 330 core

// Source 0: quad corner (±2, ±2) — per-vertex
layout(location = 0) in vec2 vertex;
// Source 1: sorted splat index — per-instance (divisor = 1)
layout(location = 8) in float uv0;

uniform samplerBuffer u_splats;   // GL_RGBA32F TBO, 19 texels per splat
uniform mat4          view_matrix;
uniform mat4          projectionMatrix;
uniform vec3          cam_pos;
uniform int           sh_degree;

out vec4 vColor;
out vec2 vPosition;

// ── TBO layout (19 vec4 texels, base = splat_id * 19) ─────────────────────────
// texel 0:  center.xyz, alpha
// texel 1:  covA.xyz (v11,v12,v13), _pad
// texel 2:  covB.xyz (v22,v23,v33), _pad
// texels 3–18: sh[0..15].rgb (16 coefficients × 3 channels, w ignored)

const float SH_C0    =  0.28209479177387814;
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

vec3 fetchSH(int base, int coeff)
{
    return texelFetch(u_splats, base + 3 + coeff).rgb;
}

vec3 evalSH(int base, vec3 d)
{
    vec3 rgb = SH_C0 * fetchSH(base, 0);

    if (sh_degree >= 1) {
        rgb += -SH_C1 * d.y * fetchSH(base, 1)
             +  SH_C1 * d.z * fetchSH(base, 2)
             + -SH_C1 * d.x * fetchSH(base, 3);
    }

    if (sh_degree >= 2) {
        float xx = d.x * d.x, yy = d.y * d.y, zz = d.z * d.z;
        float xy = d.x * d.y, yz = d.y * d.z, xz = d.x * d.z;
        rgb += SH_C2_0 * xy               * fetchSH(base, 4)
             + SH_C2_1 * yz               * fetchSH(base, 5)
             + SH_C2_2 * (2.0*zz-xx-yy)  * fetchSH(base, 6)
             + SH_C2_3 * xz               * fetchSH(base, 7)
             + SH_C2_4 * (xx - yy)        * fetchSH(base, 8);

        if (sh_degree >= 3) {
            rgb += SH_C3_0 * d.y * (3.0*xx - yy)            * fetchSH(base, 9)
                 + SH_C3_1 * d.z * xy                        * fetchSH(base, 10)
                 + SH_C3_2 * d.y * (4.0*zz - xx - yy)       * fetchSH(base, 11)
                 + SH_C3_3 * d.z * (2.0*zz - 3.0*xx-3.0*yy) * fetchSH(base, 12)
                 + SH_C3_4 * d.x * (4.0*zz - xx - yy)       * fetchSH(base, 13)
                 + SH_C3_5 * d.z * (xx - yy)                 * fetchSH(base, 14)
                 + SH_C3_6 * d.x * (xx - 3.0*yy)             * fetchSH(base, 15);
        }
    }

    return clamp(rgb + 0.5, 0.0, 1.0);
}

void main()
{
    int splat_id = int(uv0);
    int base     = splat_id * 19;

    vec4 t0  = texelFetch(u_splats, base + 0);
    vec3 center = t0.xyz;
    float alpha = t0.w;

    vec4 t1  = texelFetch(u_splats, base + 1);
    vec4 t2  = texelFetch(u_splats, base + 2);
    // Σ = {{t1.x, t1.y, t1.z}, {t1.y, t2.x, t2.y}, {t1.z, t2.y, t2.z}}

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

    // ── EWA splatting: project 3D covariance → 2D screen covariance ──────────
    mat3 J = mat3(
        projectionMatrix[0][0] / camspace.z, 0.0,
        -(projectionMatrix[0][0] * camspace.x) / (camspace.z * camspace.z),
        0.0, -projectionMatrix[1][1] / camspace.z,
        (projectionMatrix[1][1] * camspace.y) / (camspace.z * camspace.z),
        0.0, 0.0, 0.0
    );

    mat3 cov3d = mat3(
        t1.x, t1.y, t1.z,
        t1.y, t2.x, t2.y,
        t1.z, t2.y, t2.z
    );

    mat3 R   = transpose(mat3(view_matrix));
    R[0][1]  = -R[0][1];
    R[1][0]  = -R[1][0];
    mat3 T   = R * J;
    mat3 cov = transpose(T) * cov3d * T;

    vec3 vCenter = vec3(pos2d) / pos2d.w;

    float d1 = cov[0][0], od = cov[0][1], d2 = cov[1][1];
    float mid     = 0.5 * (d1 + d2);
    float radius  = length(vec2((d1 - d2) * 0.5, od));
    float lambda1 = mid + radius;
    float lambda2 = mid - radius;
    vec2 diag = normalize(vec2(od, lambda1 - d1));
    vec2 v1   = sqrt(max(lambda1, 0.0)) * diag;
    vec2 v2   = sqrt(max(lambda2, 0.0)) * vec2(diag.y, -diag.x);

    // ── SH color ──────────────────────────────────────────────────────────────
    vec3 ray_dir = normalize(center - cam_pos);
    vColor   = vec4(evalSH(base, ray_dir), alpha);
    vPosition = vertex;

    gl_Position = vec4(
        vCenter.xy + vertex.x * v1 + vertex.y * v2,
        vCenter.z, 1.0);
}
