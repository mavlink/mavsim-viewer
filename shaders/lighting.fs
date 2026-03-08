#version 330

in vec3 fragNormal;

uniform vec4 colDiffuse;
uniform vec3 lightDir;
uniform float ambient;

out vec4 finalColor;

void main() {
    vec3 n = normalize(fragNormal);
    float diff = max(dot(n, lightDir), 0.0);
    float light = ambient + (1.0 - ambient) * diff;
    finalColor = vec4(colDiffuse.rgb * light, colDiffuse.a);
}
