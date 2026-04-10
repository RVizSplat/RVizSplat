#version 120

void main()
{
    // uv in [0, 1]
    vec2 uv = gl_TexCoord[0].st;
    // distance squared from center (0.5, 0.5)
    float d2 = dot(uv - 0.5, uv - 0.5);
    // Gaussian falloff
    float alpha = exp(-16.0 * d2);
    
    // Discard fragments with very low alpha to improve performance
    if (alpha < 0.01) discard;

    gl_FragColor = vec4(gl_Color.rgb, gl_Color.a * alpha);
}
