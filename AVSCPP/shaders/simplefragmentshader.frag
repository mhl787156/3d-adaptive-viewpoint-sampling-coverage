#version 430 core

// Output data
out vec4 color;

uniform vec4 in_colour;
uniform float colorByPos; // actually boolean

void main()
{
	// Output color = userdefined
	color = in_colour;
}
