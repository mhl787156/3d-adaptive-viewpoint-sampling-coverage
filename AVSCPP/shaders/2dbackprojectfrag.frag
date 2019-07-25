#version 430 core
out ivec4 worldSpace;
  
in vec2 v_fov_scale;
in vec2 TexCoords;

uniform sampler2D texDepthWithCull;
uniform sampler2D texDepthNoCull;
uniform mat4 invViewMatrix;
uniform float scaleFactor;
uniform vec2 cameraNearFarPlane;

void main()
{
  
  vec2 depthrange = vec2(0.1, 100.0);// cameraNearFarPlane;

  // 0.1 is zNear, 100 is zFar
  float depth = 2.0 * texture(texDepthWithCull, TexCoords).x - 1.0;
  float depthNoCull = 2.0 * texture(texDepthNoCull, TexCoords).x - 1.0;

  float linear_depth = 2.0 * depthrange.x * depthrange.y / (depthrange.x + depthrange.y - depth * (depthrange.y - depthrange.x));
  float linear_depth_no_cull = 2.0 * depthrange.x * depthrange.y / (depthrange.x + depthrange.y - depthNoCull * (depthrange.y - depthrange.x));

  if ( abs(linear_depth - linear_depth_no_cull) > 0.001 ) {
    linear_depth = cameraNearFarPlane.y; // max depth
  }

  // remap Texcoords from [0, 1] to [-1, 1]
  vec2 remapscreencoord = TexCoords * 2.0 - 1.0;

  // Scale TexCoords to height and width of the near plane
  vec2 coord_on_near_plane = v_fov_scale.xy * depthrange.x * remapscreencoord;

  // Vector from eye to the near plane
  vec3 vector_eye_to_near_plane = vec3(coord_on_near_plane, -depthrange.x);

  // Vector from eye to depth (scaled near plane)
  vec3 vector_eye_to_depth = vector_eye_to_near_plane * (linear_depth / depthrange.x);

  vec4 view_space_position = vec4(vector_eye_to_depth, 1);

  vec4 worldSpc = invViewMatrix * view_space_position;
  
  worldSpace = ivec4(worldSpc.xyz * int(scaleFactor), linear_depth * int(scaleFactor));
}