#version 430 core
out ivec4 eyeSpace;
  
in vec3 eyeDirection;
in vec2 v_fov_scale;
in vec2 TexCoords;

// layout (std430, binding=2) buffer PixelLocs
// {
//   float lol;
  // vec4 pixelLocs[1280][800];
// } shader_data;


// layout(rgba32f, binding=5) uniform image2D output_image;

uniform sampler2D screenTexture;
uniform vec2 depthrange;
uniform mat4 persMatrix;
uniform mat4 invPersMatrix;
uniform mat4 invViewMatrix;


void main()
{
  float depth = texture(screenTexture, TexCoords).x;
  // eyeSpace = vec4(depth, 0, 0, 1);

  // Method 1
  // float ndcZ = (2.0 * depth - depthrange.x - depthrange.y) / (depthrange.y - depthrange.x);
  // float eyeZ = persMatrix[3][2] / ((persMatrix[2][3] * ndcZ) - persMatrix[2][2]);
  // eyeSpace =  invViewMatrix * vec4(eyeDirection * eyeZ, 1);

  // Method 2 <- This works! Just currently truncated
  vec2 half_ndc_position = vec2(0.5) - TexCoords;    // No need to multiply by two, because we already baked that into "v_tan_fov.xy".
  vec3 view_space_position = vec3(half_ndc_position * v_fov_scale.xy * -depth, -depth); // "-depth" because in OpenGL the camera is staring down the -z axis (and we're storing the unsigned depth).
  // vec4 eyeSpc = invViewMatrix * vec4(view_space_position, 1);
  vec4 eyeSpc =  vec4(view_space_position, 1);

  // int loc = int(gl_FragCoord.x) * 4 + int(gl_FragCoord.y) * 1280 * 4;
  // shader_data.lol = 1234.0;
  // shader_data.pixelLocs[int(gl_FragCoord.x)][int(gl_FragCoord.y)] = vec4(1.0, 2.0, 3.0, 4.0) ;// eyeSpace;
  // pixelLocs[loc + 1] = 2;
  // pixelLocs[loc + 2] = 3;
  // pixelLocs[loc + 3] = 4;

  // eyeSpc = vec4(0.1, 0.5, 2.0, 123.0);
  // imageStore(output_image, ivec2(gl_FragCoord.xy), eyeSpc);
  eyeSpace = ivec4(eyeSpc.xyz * 1000, 1000);

}