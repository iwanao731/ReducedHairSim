#version 400

layout(points) in;
layout(triangle_strip, max_vertices=12) out;

in vec3 gColor;
in float gIndex;
in vec3 gPosWorld[];

out vec3 fColor;
out vec3 fPosWorld;

uniform float u_fboSize;

void main(void) {
	fColor = gColor;

    float rectSize = 2.0 / u_fboSize;
    float col = mod(gIndex[0], u_fboSize) / u_fboSize * 2.0 - 1.0;
    float row = floor(gIndex[0] / u_fboSize) / u_fboSize * 2.0 - 1.0;

    vec3 v00 = vec3(col, row, 0.0);
    vec3 v01 = vec3(col + rectSize, row, 0.0);
    vec3 v10 = vec3(col, (row + rectSize), 0.0);
    vec3 v11 = vec3(col + rectSize, (row + rectSize), 0.0);

    gl_Position = vec4(v00, 1.0);
    fPosWorld = gPosWorld[0];
    EmitVertex();

    gl_Position = vec4(v01, 1.0);
    fPosWorld = gPosWorld[0];
    EmitVertex();

    gl_Position = vec4(v10, 1.0);
    fPosWorld = gPosWorld[0];
    EmitVertex();

    gl_Position = vec4(v11, 1.0);
    fPosWorld = gPosWorld[0];
    EmitVertex();

}