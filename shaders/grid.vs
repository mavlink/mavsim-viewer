#version 330

in vec3 vertexPosition;
in vec2 vertexTexCoord;
in vec4 vertexColor;

uniform mat4 mvp;
uniform mat4 matModel;

out vec3 fragWorldPos;
out vec2 fragTexCoord;
out vec4 fragColor;

void main() {
    fragWorldPos = (matModel * vec4(vertexPosition, 1.0)).xyz;
    fragTexCoord = vertexTexCoord;
    fragColor = vertexColor;
    gl_Position = mvp * vec4(vertexPosition, 1.0);
}
