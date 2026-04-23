#version 330 core

// WBOIT single-pass MRT. Both outputs use additive ONE/ONE blending:
//   location 0 : weighted colour + weight          (classic accum)
//   location 1 : -log(1-alpha) as additive transmittance proxy
// Resolve recovers revealage = exp(-sum).

uniform float wboit_weight_scale;
uniform float wboit_weight_exponent;
uniform float wboit_alpha_discard;

in  vec4  vColor;
in  vec2  vPosition;
in  float v_splat_z_warped;    // splat-tight warped depth [0,1]; -1 = use gl_FragCoord.z

layout(location = 0) out vec4 frag_accum;
layout(location = 1) out vec4 frag_reveal_log;

void main()
{
    float A = -dot(vPosition, vPosition);
    if (A < -4.0) discard;
    float alpha = exp(A) * vColor.a;
    if (alpha < wboit_alpha_discard) discard;

    // Weight uses splat-tight warped z. Raw gl_FragCoord.z for 3DGS clusters
    // in ~[0.98, 0.9998] which gives no depth separation.
    float z = (v_splat_z_warped >= 0.0) ? v_splat_z_warped : gl_FragCoord.z;
    float w = clamp(
        pow(alpha, wboit_weight_exponent) * exp(-wboit_weight_scale * z),
        1e-2, 1e4);

    frag_accum      = vec4(vColor.rgb * alpha * w, alpha * w);
    frag_reveal_log = vec4(-log(max(1.0 - alpha, 1e-4)));
}
