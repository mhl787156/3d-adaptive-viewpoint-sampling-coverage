#version 330 core
out vec4 eyeSpace;
  
in vec3 eyeDirection;
in vec2 v_fov_scale;
in vec2 TexCoords;

uniform mat4 persMatrix;
uniform vec2 depthrange;
uniform sampler2D screenTexture;

vec4 CalcEyeFromWindow(in float windowZ, in vec3 eyeDirection)
{
  float ndcZ = (2.0 * windowZ - depthrange.x - depthrange.y) /
    (depthrange.y - depthrange.x);
  float eyeZ = persMatrix[3][2] / ((persMatrix[2][3] * ndcZ) - persMatrix[2][2]);
  return vec4(eyeDirection * eyeZ, 1);
}

vec4 calculate_view_position(vec2 texture_coordinate, float depth, vec2 scale_factor)  // "scale_factor" is "v_fov_scale".
{
    vec2 half_ndc_position = vec2(0.5) - texture_coordinate;    // No need to multiply by two, because we already baked that into "v_tan_fov.xy".
    vec3 view_space_position = vec3(half_ndc_position * scale_factor.xy * -depth, -depth); // "-depth" because in OpenGL the camera is staring down the -z axis (and we're storing the unsigned depth).
    return vec4(view_space_position, 1);
}

void main()
{
  float depth = texture(screenTexture, TexCoords).x;
  eyeSpace = calculate_view_position(TexCoords, depth, v_fov_scale);
  // if (depth < 1){
  //   eyeSpace = CalcEyeFromWindow(depth, eyeDirection);
  // } else {
  //   eyeSpace = vec4(0.0, 0.0, 0.0, 0.0);
  // }

}