#version 430 core

// Input vertex data, different for all executions of this shader.
layout(location = 0) in vec4 vertexPosition_modelspace;

// Values that stay constant for the whole mesh.
uniform mat4 MVP;
uniform float scale;

void main(){
	// Output position of the vertex, in clip space : MVP * position
	gl_Position = MVP * vec4((vertexPosition_modelspace.xyz/scale), 1.0);
}

