#version 430 core

// Input vertex data, different for all executions of this shader.
layout(location = 0) in vec4 vertexPosition_modelspace;

// Values that stay constant for the whole mesh.
uniform mat4 MVP;
uniform float scale;

void main(){

	vec3 point;
	float scaler = vertexPosition_modelspace.w;
	if (scaler > 0) {
		point =  vertexPosition_modelspace.xyz / scaler; 
	} else {
		point = vec3(0.0, 0.0, 0.0);
	}

	// Output position of the vertex, in clip space : MVP * position
	gl_Position = MVP *  vec4(point, 1.0);
}

