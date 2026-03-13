#version 330

in vec2 fragBary;
in vec4 fragColor;
in vec3 fragWorldPos;

out vec4 finalColor;

void main() {
    // Reconstruct third barycentric coordinate
    float b2 = 1.0 - fragBary.x - fragBary.y;
    float minBary = min(min(fragBary.x, fragBary.y), b2);

    // Anti-aliased wireframe edge detection
    float fw = fwidth(minBary);
    float edge = 1.0 - smoothstep(0.0, fw * 1.5, minBary);

    // Glow around edges (wider, softer)
    float glow = 1.0 - smoothstep(0.0, fw * 5.0, minBary);

    // Dark fill with bright wireframe edges
    vec3 fillColor = vec3(0.015, 0.015, 0.04);
    vec3 wireColor = fragColor.rgb;
    vec3 color = mix(fillColor, wireColor, edge);
    // Add glow
    color += wireColor * glow * 0.2;

    // Distance fog — fade to sky color at horizon
    float dist = length(fragWorldPos.xz);
    vec3 fogColor = vec3(0.03, 0.03, 0.08);
    float fog = smoothstep(300.0, 800.0, dist);
    color = mix(color, fogColor, fog * 0.7);

    finalColor = vec4(color, 1.0);
}
