#version 430 core

// Output data
out vec4 color;

uniform vec4 in_colour;

void main()
{
	// Output color = userdefined
	color = in_colour;
}
