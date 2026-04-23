#version 330 core

layout(location = 0) in vec4 vertex;
layout(location = 8) in vec2 uv0;

uniform mat4 worldViewProj;

out vec2 vTexCoord;

void main()
{
    vTexCoord   = uv0;
    gl_Position = worldViewProj * vertex;
}
