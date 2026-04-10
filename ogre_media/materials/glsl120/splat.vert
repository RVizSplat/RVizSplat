#version 120

uniform mat4 worldviewproj_matrix;
uniform mat4 view_matrix;
uniform mat3 covariance3D_padded;

// Coefficients for the 2D Gaussian: Ax^2 + By^2 + Cxy
// We pass these to the fragment shader
varying vec3 conic; 

void main()
{
    gl_Position = worldviewproj_matrix * gl_Vertex;
    gl_TexCoord[0] = gl_MultiTexCoord0;
    gl_FrontColor = gl_Color;

    // 1. Transform 3D covariance to camera space
    // R is the world-to-camera rotation (top-left 3x3 of view_matrix)
    mat3 R = mat3(view_matrix[0].xyz, view_matrix[1].xyz, view_matrix[2].xyz);
    
    // Sigma_cam = R * Sigma * R^T
    mat3 sigma_cam = R * covariance3D_padded * transpose(R);
    
    // 2. Project to 2D (billboard plane)
    // We take the upper-left 2x2 block
    mat2 sigma2D = mat2(sigma_cam[0].xy, sigma_cam[1].xy);
    
    // 3. Compute Conic (inverse of 2D covariance)
    // Inverse of [a b; b c] is (1/det) * [c -b; -b a]
    float det = sigma2D[0][0] * sigma2D[1][1] - sigma2D[0][1] * sigma2D[1][0];
    
    if (det > 0.000001) {
        float invDet = 1.0 / det;
        conic.x = sigma2D[1][1] * invDet;  // A
        conic.y = sigma2D[0][0] * invDet;  // B
        conic.z = -sigma2D[0][1] * invDet; // C (x2 for the xy term)
    } else {
        conic = vec3(0.0);
    }
}
