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
uniform vec4 colFog;
uniform vec4 colTint;

out vec4 finalColor;

void main() {
    vec2 coord = fragWorldPos.xz;
    float dist = length(coord);

    // Ground color: flat or terrain-textured with distance fog
    vec4 ground = colGround;
    if (texEnabled != 0) {
        // Each world tile is 10x10 meters
        vec2 tileCoord = floor(coord / 10.0);
        vec2 tileUV = fract(coord / 10.0);

        // Hash tile coordinate to pick one of 8 atlas variants (0-7)
        float h = fract(sin(dot(tileCoord, vec2(127.1, 311.7))) * 43758.5453);
        int variant = int(h * 8.0);
        variant = clamp(variant, 0, 7);

        // Map UV into correct atlas cell (4 cols × 2 rows)
        vec2 atlasOffset = vec2(float(variant % 4) / 4.0, float(variant / 4) / 2.0);
        vec2 texUV = atlasOffset + tileUV * vec2(1.0 / 4.0, 1.0 / 2.0);

        vec4 texColor = texture(groundTex, texUV);
        // Normalize texture luminance to a detail signal centered on 1.0
        float lum = dot(texColor.rgb, vec3(0.299, 0.587, 0.114));
        float detail = (lum - 0.125) / 0.125;
        // Apply detail as brightness variation on tint color
        vec3 tinted = colTint.rgb + colTint.rgb * detail * 0.35;
        // Fog: texture visible up close, fades to fog color at distance
        float texFade = 1.0 - smoothstep(80.0, 200.0, dist);
        ground = mix(colFog, vec4(tinted, 1.0), texFade);
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
    float fade = 1.0 - smoothstep(400.0, 800.0, dist);
    color = mix(ground, color, fade);

    color.a = 1.0;
    finalColor = color;
}
