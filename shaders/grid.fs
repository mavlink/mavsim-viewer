#version 330

in vec3 fragWorldPos;
in vec2 fragTexCoord;
in vec4 fragColor;

uniform vec4 colGround;
uniform vec4 colMinor;
uniform vec4 colMajor;
uniform vec4 colAxisX;
uniform vec4 colAxisZ;
uniform float spacing;
uniform float majorEvery;
uniform float axisWidth;

uniform int texEnabled;
uniform sampler2D groundTex;
uniform vec4 colTint;
uniform vec3 camPos;

out vec4 finalColor;

void main() {
    vec2 coord = fragWorldPos.xz;
    float dist = length(coord - camPos.xz);

    // Ground color: flat or terrain-textured with camera-relative LOD
    vec4 ground = colGround;
    if (texEnabled != 0) {
        if (dist < 200.0) {
            // Near: full texture sampling — detail is visible
            vec2 tileCoord = floor(coord / 10.0);
            vec2 tileUV = fract(coord / 10.0);

            float h = fract(sin(dot(tileCoord, vec2(127.1, 311.7))) * 43758.5453);
            int variant = int(h * 8.0);
            variant = clamp(variant, 0, 7);

            vec2 atlasOffset = vec2(float(variant % 4) / 4.0, float(variant / 4) / 2.0);
            vec2 texUV = atlasOffset + tileUV * vec2(1.0 / 4.0, 1.0 / 2.0);

            vec4 texColor = texture(groundTex, texUV);
            float lum = dot(texColor.rgb, vec3(0.299, 0.587, 0.114));
            float detail = (lum - 0.125) / 0.125;
            vec3 tinted = colTint.rgb + colTint.rgb * detail * 0.35;

            // Blend from full texture to flat tint at edge of near zone
            float nearFade = smoothstep(133.0, 200.0, dist);
            ground = vec4(mix(tinted, colTint.rgb, nearFade), 1.0);
        } else {
            // Far: flat tint — zero texture reads
            ground = vec4(colTint.rgb, 1.0);
        }
    }

    // Minor grid lines
    vec2 grid = abs(fract(coord / spacing - 0.5) - 0.5) / fwidth(coord / spacing);
    float lineMinor = 1.0 - clamp(min(grid.x, grid.y), 0.0, 1.0);

    // Major grid lines
    float majorSpacing = spacing * majorEvery;
    vec2 gridMajor = abs(fract(coord / majorSpacing - 0.5) - 0.5) / fwidth(coord / majorSpacing);
    float lineMajor = 1.0 - clamp(min(gridMajor.x, gridMajor.y), 0.0, 1.0);

    // Axis lines (world origin)
    vec2 axisGrid = abs(coord) / fwidth(coord);
    float axisXLine = 1.0 - clamp(axisGrid.y / axisWidth, 0.0, 1.0); // Z=0 line (X axis)
    float axisZLine = 1.0 - clamp(axisGrid.x / axisWidth, 0.0, 1.0); // X=0 line (Z axis)

    // Compose: ground -> minor -> major -> axes
    vec4 color = ground;
    color = mix(color, colMinor, lineMinor * colMinor.a);
    color = mix(color, colMajor, lineMajor * colMajor.a);
    color = mix(color, colAxisX, axisXLine * colAxisX.a);
    color = mix(color, colAxisZ, axisZLine * colAxisZ.a);

    // Distance fade — prevent aliasing at horizon
    float fade = 1.0 - smoothstep(600.0, 1200.0, dist);
    color = mix(ground, color, fade);

    color.a = 1.0;
    finalColor = color;
}
