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
        Renderer(GLint _mWidth, GLint _mHeight);
        ~Renderer();
        void initVertexArraysandBuffers();
        void initFrameBuffers();
        void setShaders(AVSCPP::Shader &_normalShader,
                        AVSCPP::Shader &_backprojectionShader,
                        AVSCPP::Shader &_integerdisplayShader,
                        AVSCPP::CameraControl* camera);
        int* getRenderedPositions(AVSCPP::CameraControl* camera, std::vector<AVSCPP::Mesh*> modelMesh);

        bool canRender() {
            return !glfwWindowShouldClose(mWindow);
        }
        void setRenderToScreen(bool l) {renderToScreen = l;}

        GLFWwindow* getWindow(){return mWindow;}

    private:
        // timing
        float deltaTime = 0.0f;
        float lastFrame = 0.0f;
        int positionShaderScaler = 100000; // Integer to scale position outputs by

        // window
        GLint mWidth;
        GLint mHeight;
        GLFWwindow* mWindow;

        // Shaders
        AVSCPP::Shader* normalShader; // Passthrough camera render shader
        AVSCPP::Shader* backprojectionShader; // Location from depth shader
        AVSCPP::Shader* integerdisplayShader; // Integer location re-pass through shader

        // Vertex array and buffer objects
        GLuint VertexArrayObject, vbo; // For rendering "point locations" to texture
        GLuint quadVAO, quadVBO; // Target quad and buffer for rendering mesh to texture

        // Framebuffers
        GLuint framebuffer1; // Initial framebuffer for pass through render
        GLuint texColorBuffer; // Texture for collect colour information in framebuffer1
        GLuint texDepthBuffer; // Texture for collect depth information of render in framebuffer1
        GLuint framebuffer2; // 2nd framebuffer for collection position information for each rendered pixel
        GLuint projtexColorBuffer; // Texture for collecting x,y,z position as scaled integers (could not pass data back as floats due to truncation)

        // DataStore
        GLint * pixelArray;
        bool renderToScreen = false;




};


}
#endif