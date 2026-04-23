#version 330 core

uniform float u_alpha_threshold;

in  vec4 vColor;
in  vec2 vPosition;
out vec4 frag_color;

void main()
{
    float A = -dot(vPosition, vPosition);
    float B = exp(A) * vColor.a;
    if (B < u_alpha_threshold) discard;
    // Pre-multiplied alpha: RGB already scaled, compositor blends with one/one_minus_src_alpha
    frag_color = vec4(vColor.rgb * B, B);
}
