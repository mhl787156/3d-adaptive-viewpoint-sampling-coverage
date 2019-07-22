// Local Headers
#include "avscpp.hpp"

// System Headers
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/string_cast.hpp>

// Opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

// Standard Headers
#include <cstdio>
#include <cstdlib>
#include <utility> 
#include <iostream>
#include <fstream>

// Local Library Headers
#include "render.hpp"
#include "shader.hpp"
#include "mesh.hpp"
#include "controls.hpp"



int main(int argc, char * argv[]) {
    
    // init
    AVSCPP::Renderer renderer(mWidth, mHeight);
    renderer.setRenderToScreen(true);

    // Read in .obj filec
    std::vector<AVSCPP::Mesh*> meshes;

    std::string const modelPath = PROJECT_SOURCE_DIR "/resources/models/aeroplane.obj";
    AVSCPP::Mesh modelPoints(modelPath);
    glm::mat4 locmat = glm::mat4(1.0);
    locmat[3] = glm::vec4(0.0, 1.0, 0.0, 1.0);
    modelPoints.setModelMatrix(locmat);
    meshes.push_back(&modelPoints);

    std::string const floorPath = PROJECT_SOURCE_DIR "/resources/models/plane.obj";
    AVSCPP::Mesh floorMesh(floorPath);
    floorMesh.setModelMatrix(glm::mat4(1.0));
    meshes.push_back(&floorMesh);

    // Create Model-View-Projection Matrix:
    AVSCPP::CameraControl camera(renderer.getWindow());

    // Create and compile our GLSL program from the shaders
    std::string shaderPath = PROJECT_SOURCE_DIR "/AVSCPP/shaders/";
    AVSCPP::Shader shader;
    shader.attach(shaderPath+"simplevertexshader.vert")
          .attach(shaderPath+"simplefragmentshader.frag")
          .link();

    AVSCPP::Shader intshader;
    intshader.attach(shaderPath+"simplevertexshaderint.vert")
          .attach(shaderPath+"simplefragmentshaderint.frag")
          .link();

    AVSCPP::Shader projshader;
    projshader.attach(shaderPath+"2dbackprojectvert.vert")
              .attach(shaderPath+"2dbackprojectfrag.frag")
              .link();

    renderer.setShaders(shader, projshader, intshader, &camera);

    renderer.initVertexArraysandBuffers();

    renderer.initFrameBuffers();

    while(renderer.canRender()) {
        GLint* pixelLocs = renderer.getRenderedPositions(&camera, meshes);
    }


    return EXIT_SUCCESS;
}
