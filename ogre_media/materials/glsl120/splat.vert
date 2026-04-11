#version 120

uniform mat4 worldviewproj_matrix;
uniform mat4 view_matrix, projectionMatrix;
uniform mat3 covariance3D;
uniform vec2 focal;
uniform vec3 cam_pos;
uniform vec4 viewportSize;
uniform vec3 splat_center;
uniform vec4 splat_color;

varying vec4 vColor;
varying vec2 vPosition;

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

    // 1. Transform 3D covariance to camera space
    // W is the transpose of world-to-camera rotation (top-left 3x3 of view_matrix)
    mat3 W = transpose(mat3(view_matrix));

    mat3 J = mat3(
        projectionMatrix[0][0] / camspace.z, 0.0,
        -(projectionMatrix[0][0] * camspace.x) / (camspace.z * camspace.z),

        0.0, -projectionMatrix[1][1] / camspace.z,
        (projectionMatrix[1][1] * camspace.y) / (camspace.z * camspace.z),

        0.0, 0.0, 0.0
    );
    
    mat3 T = W * J;

    // Sigma_cam = R * Sigma * R^T
    mat3 cov = transpose(T) * covariance3D * T;
    
    vec2 vCenter = vec2(pos2d) / pos2d.w;

    float diagonal1 = cov[0][0] + 0.3;
    float offDiagonal = cov[0][1];
    float diagonal2 = cov[1][1] + 0.3;

    float mid = 0.5 * (diagonal1 + diagonal2);
    float radius = length(vec2((diagonal1 - diagonal2) / 2.0, offDiagonal));
    float lambda1 = mid + radius;
    float lambda2 = max(mid - radius, 0.1);
    vec2 diagonalvector = normalize(vec2(offDiagonal, lambda1 - diagonal1));
    vec2 v1 = min(sqrt(2.0 * lambda1), 1024.0) * diagonalvector;
    vec2 v2 = min(sqrt(2.0 * lambda2), 1024.0) * vec2(diagonalvector.y, -diagonalvector.x);

    // vec3 ray_direction = normalize(splat_center - cam_pos);
    vColor.rgb = splat_color.rgb;
    // vColor.rgb = get_rgb(ray_direction); // Get spherical harmonics here
    vColor.a = splat_color.a;
    vPosition = gl_Vertex.xy;

    vec2 viewport = viewportSize.xy;
    gl_Position = vec4(
        vCenter
            + (vPosition.x) * v1 / viewport * 2.0
            + (vPosition.y) * v2 / viewport * 2.0, 0.0, 1.0);
}
