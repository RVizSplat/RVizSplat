#version 330 core

in  vec4 vColor;
in  vec2 vPosition;
out vec4 frag_color;

void main()
{
    float A = -dot(vPosition, vPosition);
    if (A < -4.0) discard;
    float B = exp(A) * vColor.a;
    // Pre-multiplied alpha: RGB already scaled, compositor blends with one/one_minus_src_alpha
    frag_color = vec4(vColor.rgb * B, B);
}
