#version 330 core
layout (location = 0) in vec2 aPos;
layout (location = 1) in vec2 aTexCoords;

out vec3 eyeDirection;
out vec2 v_fov_scale;
out vec2 TexCoords;

// Half the size of the near plane {tan(fovy/2.0) * aspect, tan(fovy/2.0) }
uniform vec2 halfSizeNearPlane; 

void main()
{
    gl_Position = vec4(aPos.x, aPos.y, 0.0, 1.0); 
    eyeDirection = vec3((2.0 * halfSizeNearPlane * aTexCoords) - halfSizeNearPlane , -1.0);
    v_fov_scale = halfSizeNearPlane * 2.0;
    TexCoords = aTexCoords;    
}  