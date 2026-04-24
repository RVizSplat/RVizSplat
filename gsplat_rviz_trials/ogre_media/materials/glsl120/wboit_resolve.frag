#version 330 core

// Resolve pass for single-MRT WBOIT.
// accum_tex     : .rgb = Σ wᵢ αᵢ RGBᵢ,  .a = Σ wᵢ αᵢ
// reveal_log_tex: .r   = Σ -ln(1 - αᵢ)
// revealage = exp(-reveal_log) = Π(1 - αᵢ).

uniform sampler2D scene_tex;
uniform sampler2D accum_tex;
uniform sampler2D reveal_log_tex;

in  vec2 vTexCoord;
out vec4 frag_color;

void main()
{
    vec4  scene      = texture(scene_tex, vTexCoord);
    vec4  accum      = texture(accum_tex, vTexCoord);
    float reveal_log = texture(reveal_log_tex, vTexCoord).r;
    float revealage  = exp(-reveal_log);

    // Weighted average premultiplied colour, then Porter-Duff OVER.
    vec3 avg_color = accum.rgb / max(accum.a, 1e-5);
    vec3 result    = avg_color * (1.0 - revealage) + scene.rgb * revealage;

    frag_color = vec4(result, 1.0);
}
