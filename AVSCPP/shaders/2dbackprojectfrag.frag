#version 430 core
out ivec4 worldSpace;
  
in vec3 eyeDirection;
in vec2 v_fov_scale;
in vec2 TexCoords;

uniform sampler2D screenTexture;
uniform mat4 invViewMatrix;
uniform mat4 persMatrix;
uniform vec3 eyePositioninWorld;
uniform float scaleFactor;
uniform vec2 cameraNearFarPlane;

void main()
{
  
  vec2 depthrange = vec2(0.1, 100.0);
  // 0.1 is zNear, 100 is zFar
  float depth = texture(screenTexture, TexCoords).x;
  float depthsample = 2.0 * depth - 1.0;
  float linear_depth = 2.0 * 0.1 * 100.0 / (0.1 + 100.0 - depthsample * (100.0 - 0.1));

  // remap Texcoords from [0, 1] to [-1, 1]
  vec2 remapscreencoord = TexCoords * 2 - 1.0;

  // Scale TexCoords to height and width of the near plane
  vec2 coord_on_near_plane = v_fov_scale.xy * 0.1 * remapscreencoord;

  // Scale TexCoords to height and width of the near plane
  vec2 coord_on_far_plane = v_fov_scale.xy * 100.0 * remapscreencoord;

  // Vector from eye to the near plane
  vec3 vector_eye_to_near_plane = vec3(coord_on_near_plane, -0.1);

  // Vector from eye to depth (scaled near plane)
  vec3 vector_eye_to_depth = vector_eye_to_near_plane * (linear_depth / depthrange.x);

  // Vector from eye to the far plane
  // vec3 vector_eye_to_far_plane = vec3(coord_on_far_plane, -100.0);

  // Scale vector to depth
  // vec3 view_space_position = vec3(vector_eye_to_far_plane.xy * linear_depth, -linear_depth);
  vec4 view_space_position = vec4(vector_eye_to_depth, 1);

  // float ndcZ = (2.0 * depth - depthrange.x - depthrange.y) /
  // (depthrange.y - depthrange.x);
  // float eyeZ = persMatrix[3][2] / ((persMatrix[2][3] * ndcZ) - persMatrix[2][2]);
  // view_space_position = eyeDirection * eyeZ;


  // vec2 half_ndc_position = vec2(0.5) - TexCoords;    // No need to multiply by two, because we already baked that into "v_tan_fov.xy".
  // vec3 view_space_position = vec3(half_ndc_position * v_fov_scale.xy * -linear_depth, -linear_depth); // "-depth" because in OpenGL the camera is staring down the -z axis (and we're storing the unsigned depth).
  // vec3 worldSpc = eyePositioninWorld + view_space_position;
  vec4 worldSpc = invViewMatrix * view_space_position;
  
  worldSpace = ivec4(worldSpc.xyz * int(scaleFactor), int(scaleFactor));
}