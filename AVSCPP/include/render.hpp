#ifndef RENDER_AVSCPP_HPP
#define RENDER_AVSCPP_HPP

// System Headers
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/string_cast.hpp>

// Standard Headers
#include <cstdio>
#include <cstdlib>
#include <utility> 
#include <iostream>
#include <fstream>
#include <vector>

// Local Library Headers
#include "avscpp.hpp"
#include "shader.hpp"
#include "mesh.hpp"
#include "controls.hpp"

void processInput(GLFWwindow *window, AVSCPP::CameraControl *c);
GLfloat pixelIndex(GLfloat *b, int w, int h, int c);
GLint pixelIndex(GLint *b, int w, int h, int c);

int* renderCameraView(std::vector<float> cameraloc, std::vector<float> cameraorient);



namespace AVSCPP {

class Renderer {

    public:
        int render();

    private:
            // timing
        float deltaTime = 0.0f;
        float lastFrame = 0.0f;

};


}
#endif