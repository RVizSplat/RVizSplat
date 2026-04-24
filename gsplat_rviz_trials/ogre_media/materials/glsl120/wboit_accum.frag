#version 330 core

// WBOIT single-pass MRT (McGuire-Bavoil, log-space reveal variant).
// Both attachments use additive ONE/ONE blending.
//
//   location 0 (accum)      : Σ wᵢ · αᵢ · RGBᵢ     (.rgb)
//                             Σ wᵢ · αᵢ              (.a, total weight)
//   location 1 (reveal_log) : Σ -ln(1 - αᵢ)         (.r,  scalar channel)
//
// The resolve pass recovers revealage = exp(-reveal_log), which is
// algebraically identical to the classical Π(1 - αᵢ) reveal.
// Storing the sum additively (rather than the product multiplicatively)
// lets both attachments share a single blend equation, which is the
// prerequisite for MRT fusion.

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

    // Weight: front fragments and high-alpha fragments dominate.
    // Raw gl_FragCoord.z for 3DGS clusters in ~[0.98, 0.9998] gives no depth
    // separation — the splat-tight warped z fixes that when available.
    float z = (v_splat_z_warped >= 0.0) ? v_splat_z_warped : gl_FragCoord.z;
    float w = clamp(
        pow(alpha, wboit_weight_exponent) * exp(-wboit_weight_scale * z),
        1e-2, 1e4);

    frag_accum = vec4(vColor.rgb * alpha * w, alpha * w);

    // Log-space transmittance. The (1 - α) floor prevents -log(0) for α≈1;
    // 1e-2 leaves ~8.7 ulps of FP16 headroom at the clamp and keeps
    // near-opaque fragments from hitting the +∞ rail. The clamp only
    // affects fragments with α > 0.99, for which revealage is already
    // effectively zero and the clamp is invisible in the composite.
    frag_reveal_log = vec4(-log(max(1.0 - alpha, 1e-2)));
}
