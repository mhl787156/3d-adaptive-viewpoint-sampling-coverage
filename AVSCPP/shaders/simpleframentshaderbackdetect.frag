#version 430 core

// Output data
out vec4 color;

void main()
{
    if(!gl_FrontFacing) { // If backfacing, set alpha channel to negative
        color = vec4(0.0, 0.0, 1.0, 0.0);
    } else {
        color = vec4(1.0, 0.0, 0.0, 1.0);
    }
}