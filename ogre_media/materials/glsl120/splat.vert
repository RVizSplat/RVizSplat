#version 120

uniform mat4 worldviewproj_matrix;
uniform mat4 view_matrix, projectionMatrix;
uniform mat3 covariance3D;
uniform vec2 focal;
uniform vec3 cam_pos;
uniform vec4 viewportSize;
uniform vec3 splat_center;
uniform vec4 splat_color;

// Spherical harmonics coefficients in coefficient-major order.
// spherical_harmonics[i] = (R, G, B) contribution of SH basis function i.
// Index 0 is the DC (constant) term; 1–15 are view-dependent (degrees 1–3).
uniform vec3 spherical_harmonics[16];
uniform int  sh_degree;

varying vec4 vColor;
varying vec2 vPosition;

const float SH_C0 = 0.28209479177387814;
const float SH_C1 = 0.4886025119029199;
const float SH_C2_0 =  1.0925484305920792;
const float SH_C2_1 = -1.0925484305920792;
const float SH_C2_2 =  0.31539156525252005;
const float SH_C2_3 = -1.0925484305920792;
const float SH_C2_4 =  0.5462742152960396;
const float SH_C3_0 = -0.5900435899266435;
const float SH_C3_1 =  2.890611442640554;
const float SH_C3_2 = -0.4570457994644658;
const float SH_C3_3 =  0.3731763325901154;
const float SH_C3_4 = -0.4570457994644658;
const float SH_C3_5 =  1.445305721320277;
const float SH_C3_6 = -0.5900435899266435;

// Evaluate SH up to sh_degree for view direction d (unit vector).
// Uses the spherical_harmonics uniform array populated from the PLY loader.
vec3 get_rgb(vec3 d)
{
    vec3 rgb = SH_C0 * spherical_harmonics[0];

    if (sh_degree >= 1) {
        rgb +=
            - SH_C1 * d.y * spherical_harmonics[1]
            + SH_C1 * d.z * spherical_harmonics[2]
            - SH_C1 * d.x * spherical_harmonics[3];
    }

    if (sh_degree >= 2) {
        float xx = d.x * d.x;
        float yy = d.y * d.y;
        float zz = d.z * d.z;
        float xy = d.x * d.y;
        float yz = d.y * d.z;
        float xz = d.x * d.z;
        rgb +=
            SH_C2_0 * xy              * spherical_harmonics[4] +
            SH_C2_1 * yz              * spherical_harmonics[5] +
            SH_C2_2 * (2.0*zz-xx-yy) * spherical_harmonics[6] +
            SH_C2_3 * xz              * spherical_harmonics[7] +
            SH_C2_4 * (xx - yy)       * spherical_harmonics[8];

        if (sh_degree >= 3) {
            rgb +=
                SH_C3_0 * d.y * (3.0*xx - yy)           * spherical_harmonics[9]  +
                SH_C3_1 * d.z * xy                       * spherical_harmonics[10] +
                SH_C3_2 * d.y * (4.0*zz - xx - yy)      * spherical_harmonics[11] +
                SH_C3_3 * d.z * (2.0*zz - 3.0*xx-3.0*yy)* spherical_harmonics[12] +
                SH_C3_4 * d.x * (4.0*zz - xx - yy)      * spherical_harmonics[13] +
                SH_C3_5 * d.z * (xx - yy)                * spherical_harmonics[14] +
                SH_C3_6 * d.x * (xx - 3.0*yy)            * spherical_harmonics[15];
        }
    }

    return clamp(rgb + 0.5, 0.0, 1.0);
}

void main()
{
    // Get the center of splat in cam frame
    vec4 camspace = view_matrix * vec4(splat_center, 1.0);
    vec4 pos2d = projectionMatrix * camspace;

    // Make the splat disappear if behind cam or far from frustum
    float bounds = 1.2 * pos2d.w;
    if (pos2d.z < -pos2d.w
        || pos2d.x < -bounds
        || pos2d.x > bounds
        || pos2d.y < -bounds
        || pos2d.y > bounds) {
        gl_Position = vec4(0.0, 0.0, 2.0, 1.0);
        return;
    }

    mat3 J = mat3(
        projectionMatrix[0][0] / camspace.z, 0.0,
        -(projectionMatrix[0][0] * camspace.x) / (camspace.z * camspace.z),

        0.0, -projectionMatrix[1][1] / camspace.z,
        (projectionMatrix[1][1] * camspace.y) / (camspace.z * camspace.z),

        0.0, 0.0, 0.0
    );

    mat3 R = transpose(mat3(view_matrix));
    R[0][1] = -R[0][1];
    R[1][0] = -R[1][0];
    mat3 T = R * J;
    mat3 cov = transpose(T) * covariance3D * T;

    vec2 vCenter = vec2(pos2d) / pos2d.w;

    float diagonal1   = cov[0][0];
    float offDiagonal = cov[0][1];
    float diagonal2   = cov[1][1];

    float mid    = 0.5 * (diagonal1 + diagonal2);
    float radius = length(vec2((diagonal1 - diagonal2) / 2.0, offDiagonal));
    float lambda1 = mid + radius;
    float lambda2 = mid - radius;
    vec2 diagonalvector = normalize(vec2(offDiagonal, lambda1 - diagonal1));
    vec2 v1 = sqrt(lambda1) * diagonalvector;
    vec2 v2 = sqrt(lambda2) * vec2(diagonalvector.y, -diagonalvector.x);

    // View-dependent color via spherical harmonics.
    vec3 ray_direction = normalize(splat_center - cam_pos);
    vColor.rgb = get_rgb(ray_direction);
    vColor.a   = splat_color.a;
    vPosition  = gl_Vertex.xy;

    gl_Position = vec4(
        vCenter
            + vPosition.x * v1
            + vPosition.y * v2, 0.0, 1.0);
}
