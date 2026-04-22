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
//   texel 1.z   = packUnorm4x8(r, g, b, a)
//   texel 1.w   = reserved (SH index, used in SH>=1 path)
uniform usamplerBuffer u_splats;
uniform mat4           view_matrix;
uniform mat4           projectionMatrix;

out vec4 vColor;
out vec2 vPosition;

void main()
{
    int splat_id = int(uv0);
    int base     = splat_id * 2;

    uvec4 t0 = texelFetch(u_splats, base + 0);
    uvec4 t1 = texelFetch(u_splats, base + 1);

    vec3 center = uintBitsToFloat(t0.xyz);

    vec2 c0001 = unpackHalf2x16(t0.w);
    vec2 c0211 = unpackHalf2x16(t1.x);
    vec2 c1222 = unpackHalf2x16(t1.y);

    float c00 = c0001.x, c01 = c0001.y;
    float c02 = c0211.x, c11 = c0211.y;
    float c12 = c1222.x, c22 = c1222.y;

    vec4 rgba = unpackUnorm4x8(t1.z);

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

    vColor    = rgba;   // DC-only color pre-baked as uint8 on upload
    vPosition = vertex;

    gl_Position = vec4(
        vCenter.xy + vertex.x * v1 + vertex.y * v2,
        vCenter.z, 1.0);
}
