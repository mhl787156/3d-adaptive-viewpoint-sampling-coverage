#version 430 core

// Output data
out vec4 color;

uniform vec4 in_colour;

void main()
{

	// Output color = red 
	color = in_colour; //vec4(1.0, 0.0, 0.0, 1.0);

}
