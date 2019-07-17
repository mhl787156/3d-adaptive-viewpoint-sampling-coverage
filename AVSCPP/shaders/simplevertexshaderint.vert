#version 430 core

// Input vertex data, different for all executions of this shader.
layout(location = 0) in ivec4 vertexPosition_modelspace;

// Vertex Output Data
// out vec3 vcolour;

// Values that stay constant for the whole mesh.
uniform mat4 MVP;
uniform float scale;

void main(){

	gl_PointSize = 3.0;

	vec3 point = vec3(vertexPosition_modelspace.xyz);
	float scaler = float(vertexPosition_modelspace.w);
	if (scaler > 0) {
		point =  point / scaler; 
	} else {
		point = vec3(0.0, 0.0, 0.0);
	}

	// Output position of the vertex, in clip space : MVP * position
	gl_Position =  MVP * vec4(point, 1.0);
	// vcolour = vec3(1.0, 0.0, 0.0); //vertexPosition_modelspace;
}

