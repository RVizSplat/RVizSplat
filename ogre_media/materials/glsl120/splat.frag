#version 120

// conic.x = A, conic.y = B, conic.z = C
// corresponding to Ax^2 + By^2 + 2Cxy
varying vec3 conic;

void main()
{
    // Local coordinates in [-2, 2] (since quad is 4x4)
    // Map UV [0, 1] to [-2, 2]
    vec2 d = (gl_TexCoord[0].st - 0.5) * 4.0;
    
    // Evaluate Gaussian: power = -0.5 * (Ax^2 + By^2 + 2Cxy)
    float power = -0.5 * (conic.x * d.x * d.x + conic.y * d.y * d.y + 2.0 * conic.z * d.x * d.y);
    
    if (power > 0.0) discard; // Should not happen for valid covariance
    
    float alpha = exp(power);
    
    if (alpha < 0.01) discard;

    gl_FragColor = vec4(gl_Color.rgb, gl_Color.a * alpha);
}
