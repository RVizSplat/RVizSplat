#version 330 core

uniform sampler2D scene_tex;
uniform sampler2D accum_tex;
uniform sampler2D reveal_log_tex;  // additive Σ -log(1-α); revealage = exp(-r)

in  vec2 vTexCoord;
out vec4 frag_color;

void main()
{
    vec4  scene      = texture(scene_tex, vTexCoord);
    vec4  accum      = texture(accum_tex, vTexCoord);
    float reveal_log = texture(reveal_log_tex, vTexCoord).r;
    float revealage  = exp(-reveal_log);

    // Weighted average transparent colour, then composite.
    vec3 avg_color = accum.rgb / max(accum.a, 1e-5);
    vec3 result    = avg_color * (1.0 - revealage) + scene.rgb * revealage;

    frag_color = vec4(result, 1.0);
}
