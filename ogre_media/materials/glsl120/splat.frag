#version 120

varying vec4 vColor;
varying vec2 vPosition;

void main ()
{    
    float A = -dot(vPosition, vPosition);
    if (A < -4.0) discard;
    float B = exp(A) * vColor.a;
    gl_FragColor = vec4(vColor.rgb, B);
    // gl_FragColor = vec4(B * vColor.rgb, B); // Is more optimized
    // gl_FragColor = vec4(vColor.rgb, 1.0);
}
