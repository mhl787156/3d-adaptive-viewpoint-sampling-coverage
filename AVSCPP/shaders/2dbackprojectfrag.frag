#version 430 core
out ivec4 eyeSpace;
  
in vec3 eyeDirection;
in vec2 v_fov_scale;
in vec2 TexCoords;

uniform sampler2D screenTexture;
uniform mat4 invViewMatrix;
uniform float scaleFactor;

void main()
{
  float depth = texture(screenTexture, TexCoords).x;

  vec2 half_ndc_position = vec2(0.5) - TexCoords;    // No need to multiply by two, because we already baked that into "v_tan_fov.xy".
  vec3 view_space_position = vec3(half_ndc_position * v_fov_scale.xy * -depth, -depth); // "-depth" because in OpenGL the camera is staring down the -z axis (and we're storing the unsigned depth).
  vec4 eyeSpc = invViewMatrix * vec4(view_space_position, 1);

  eyeSpace = ivec4(eyeSpc.xyz * int(scaleFactor), int(scaleFactor));

}