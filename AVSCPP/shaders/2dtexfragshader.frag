#version 330 core
out vec4 FragColor;
  
in vec2 TexCoords;

uniform sampler2D screenTexture;

void main()
{ 
    float depth = texture(screenTexture, TexCoords).x;
    if(depth<1) {
        FragColor = vec4(depth, depth, 0, 1.0);
    }
}