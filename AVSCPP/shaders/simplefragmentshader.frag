#version 430 core

layout(std430, binding=3) buffer sso
{
	vec4 shader_data[];
};

// Output data
out vec4 color;

void main()
{

	shader_data[0].x = gl_PrimitiveID;
	shader_data[0].y = gl_SampleID;
	shader_data[0].z = gl_SampleID;

	// Output color = red 
	color = vec4(1.0, 0.0, 0.0, 1.0);

}
