#version 330 core

uniform sampler2D scene_tex;
uniform sampler2D accum_tex;
uniform sampler2D reveal_tex;

in  vec2 vTexCoord;
out vec4 frag_color;

void main()
{
    vec4  scene     = texture(scene_tex, vTexCoord);
    vec4  accum     = texture(accum_tex, vTexCoord);
    float revealage = texture(reveal_tex, vTexCoord).r;

    // Average transparent color = weighted sum / total weight.
    vec3 avg_color = accum.rgb / max(accum.a, 1e-5);

    // Composite: transparent contribution * coverage + scene * transmittance
    vec3 result = avg_color * (1.0 - revealage) + scene.rgb * revealage;

    frag_color = vec4(result, 1.0);
}
