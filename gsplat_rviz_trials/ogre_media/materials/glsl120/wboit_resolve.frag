#version 330 core

// Resolve pass for single-MRT WBOIT.
//
// DEBUG BUILD — this shader is currently instrumented for diagnosing a
// reported black-screen regression.  The four-quadrant diagnostic overlay
// will be removed once we identify which stage (scene / accum / reveal /
// resolve) is failing.

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

    // --- Diagnostic overlay --------------------------------------------
    //
    // Split the viewport into four quadrants so the on-screen image tells
    // us which stage of the pipeline is producing output:
    //
    //   top-left   (uv.x<0.5 && uv.y>0.5) : scene_tex.rgb        — pass 1
    //   top-right  (uv.x>0.5 && uv.y>0.5) : accum_tex.rgb * 50   — pass 2 [0]
    //   bottom-left                        : reveal_log * 0.5    — pass 2 [1]
    //   bottom-right                       : normal WBOIT resolve
    //
    // A bright red border on all four quadrants confirms this resolve pass
    // actually ran.  If the whole screen is still black, the resolve pass
    // itself isn't executing and the compositor chain is the problem.
    // ------------------------------------------------------------------
    vec3 debug;
    if (vTexCoord.x < 0.5 && vTexCoord.y > 0.5) {
        debug = scene.rgb;
    } else if (vTexCoord.x >= 0.5 && vTexCoord.y > 0.5) {
        debug = accum.rgb * 50.0;
    } else if (vTexCoord.x < 0.5 && vTexCoord.y <= 0.5) {
        debug = vec3(reveal_log * 0.5);
    } else {
        float revealage = exp(-reveal_log);
        vec3  avg_color = accum.rgb / max(accum.a, 1e-5);
        debug = avg_color * (1.0 - revealage) + scene.rgb * revealage;
    }

    // Red 1-px border at each quadrant boundary.
    vec2 px = 1.0 / vec2(textureSize(scene_tex, 0));
    float border = 0.0;
    if (abs(vTexCoord.x - 0.5) < px.x) border = 1.0;
    if (abs(vTexCoord.y - 0.5) < px.y) border = 1.0;

    frag_color = vec4(mix(debug, vec3(1.0, 0.0, 0.0), border), 1.0);
}
