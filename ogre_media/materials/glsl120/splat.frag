#version 120

varying vec4 vColor;
varying vec2 vPosition;

void main ()
{    
    float A = -dot(vPosition, vPosition);
    if (A < -4.0) discard;
    float B = exp(A) * vColor.a;

    // Gaussian Visualisation
    gl_FragColor = vec4(vColor.rgb, B);
    // gl_FragColor = vec4(B * vColor.rgb, B); // Is more optimized

    // Eigen vectors visualisation
    // vec3 color_blue = vec3(0., 0., 1.);
    // vec3 color_red = vec3(1., 0., 0.);
    // float scale_factor = (vPosition.x + 2.) / (4.);
    //
    // if (abs(vPosition.x) < 1e-2) {
    //   float alpha_scale = (vPosition.y + 1.) / 2.;
    //   gl_FragColor = vec4(0., 0., 0., alpha_scale);
    //   return;
    // }
    // if (abs(vPosition.y) < 1e-2) {
    //   float alpha_scale = (vPosition.x + 1.) / 2.;
    //   gl_FragColor = vec4(1., 1., 1., alpha_scale);
    //   return;
    // }
    // vec3 final_color = color_blue * scale_factor + (1. - scale_factor) * color_red;
    // gl_FragColor = vec4(final_color, 0.5);
    
    // Solid color visualisation
    // gl_FragColor = vec4(vColor.rgb, 1.0);
}
