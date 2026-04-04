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

// Hash helpers
vec2 hash2(vec2 p) {
    return vec2(
        fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453),
        fract(sin(dot(p, vec2(269.5, 183.3))) * 43758.5453)
    );
}

// Grid line type: 0 = L (light), 1 = D (dark)
int gridLineType(float idx) {
    return fract(sin(idx * 127.1) * 43758.5453) > 0.5 ? 1 : 0;
}

void main() {
    vec2 coord = fragWorldPos.xz;
    float dist = length(coord - camPos.xz);

    // Ground color: flat or terrain-textured with camera-relative LOD
    vec4 ground = colGround;
    if (texEnabled != 0) {
        if (dist < 200.0) {
            // Near: full texture sampling — detail is visible
            float cellSize = 3.0;
            float warpStrength = 0.75;

            // Vertex wobble: per-vertex displacement, bilinearly interpolated
            vec2 cell = floor(coord / cellSize);
            vec2 f = fract(coord / cellSize);
            vec2 d00 = (hash2(cell) * 2.0 - 1.0) * warpStrength;
            vec2 d10 = (hash2(cell + vec2(1.0, 0.0)) * 2.0 - 1.0) * warpStrength;
            vec2 d01 = (hash2(cell + vec2(0.0, 1.0)) * 2.0 - 1.0) * warpStrength;
            vec2 d11 = (hash2(cell + vec2(1.0, 1.0)) * 2.0 - 1.0) * warpStrength;
            vec2 disp = mix(mix(d00, d10, f.x), mix(d01, d11, f.x), f.y);
            vec2 warped = coord + disp;

            vec2 tileCoord = floor(warped / cellSize);
            vec2 tileUV = fract(warped / cellSize);

            // Runtime edge matching via grid-line types.
            // Each grid line (vertical and horizontal) is hashed to L(0) or D(1).
            // The cell's 4 edges inherit from its bounding grid lines.
            // Atlas row = edge group = top*8 + right*4 + bottom*2 + left.
            int eTop    = gridLineType(tileCoord.y);
            int eBottom = gridLineType(tileCoord.y + 1.0);
            int eLeft   = gridLineType(tileCoord.x);
            int eRight  = gridLineType(tileCoord.x + 1.0);

            // Random flip: swap UV axes and adjust edge group accordingly
            float flipH = fract(sin(dot(tileCoord, vec2(53.14, 197.3))) * 43758.5453);
            float flipV = fract(sin(dot(tileCoord, vec2(91.72, 43.87))) * 43758.5453);
            int tmp;
            if (flipH > 0.5) {
                tileUV.x = 1.0 - tileUV.x;
                tmp = eLeft; eLeft = eRight; eRight = tmp;
            }
            if (flipV > 0.5) {
                tileUV.y = 1.0 - tileUV.y;
                tmp = eTop; eTop = eBottom; eBottom = tmp;
            }

            int group = eTop * 8 + eRight * 4 + eBottom * 2 + eLeft;

            // Pick variant within group (14 per group)
            float h = fract(sin(dot(tileCoord, vec2(127.1, 311.7))) * 43758.5453);
            int local = int(h * 14.0);
            local = clamp(local, 0, 13);

            // Atlas: 14 columns × 16 rows, column = variant, row = group
            vec2 atlasOffset = vec2(float(local) / 14.0, float(group) / 16.0);
            vec2 texUV = atlasOffset + tileUV * vec2(1.0 / 14.0, 1.0 / 16.0);

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
