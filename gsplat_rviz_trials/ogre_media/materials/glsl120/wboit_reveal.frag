#version 330 core

// WBOIT revealage pass. Blend ZERO/ONE_MINUS_SRC_COLOUR against buffer
// cleared to 1 → destination accumulates ∏(1 - alpha_i).

in  vec4 vColor;
in  vec2 vPosition;
out vec4 frag_color;

void main()
{
    float A = -dot(vPosition, vPosition);
    if (A < -4.0) discard;
    float alpha = exp(A) * vColor.a;
    frag_color = vec4(alpha);
}
