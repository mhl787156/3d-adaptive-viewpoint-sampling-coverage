// Local Headers
#include "main.hpp"

#include <chrono>
#include <thread>

// // Opencv
// #include <opencv2/opencv.hpp>
// #include <opencv2/core.hpp>
// #include <opencv2/highgui.hpp>

bool renderToScreen = false;
bool setDebug = false;
bool renderTesting = false; 
int waitMillis = 0;

// Function declarations
std::vector<AVSCPP::Mesh*> loadMeshes();
void loadShaders(AVSCPP::Renderer &renderer, AVSCPP::CameraControl &camera);

int main(int argc, char * argv[]) {

    for (int i = 1; i < argc; i++) {
        if(std::string(argv[i]) == "-d") {setDebug = true; continue;}
        if(std::string(argv[i]) == "-s") {renderToScreen = true; continue;}
        if(std::string(argv[i]) == "-rt") {renderTesting = true; continue;}
        if(std::string(argv[i]) == "-w") {waitMillis = std::stoi(std::string(argv[++i])); continue;}
    }
    
    // Init OpenGL
    AVSCPP::Renderer renderer(mWidth, mHeight);
    renderer.setRenderToScreen(renderToScreen);
    renderer.setDebug(setDebug);

    // Create Model-View-Projection Matrix:
    AVSCPP::CameraControl camera(renderer.getWindow());

    // Read in .obj filec
    std::vector<AVSCPP::Mesh*> meshes = loadMeshes();

    // Load Shaders
    loadShaders(renderer, camera);

    // Complete OpenGL setup by loading buffers
    renderer.initVertexArraysandBuffers();
    renderer.initFrameBuffers();

    if(renderTesting){ // Just for rendering only!
        while(renderer.canRender()) {
        renderer.getRenderedPositions(camera, meshes);
        }
        return EXIT_SUCCESS;
    }


    AVSCPP::CoveragePlanner planner(renderer, camera, meshes);
    planner.setDebug(setDebug);

    // Planner renders a set of viewpoints using a default resolution
    // passing an empty vector will cause the full Mesh bounding box to be used.
    planner.sampleViewpoints(nullptr, 10.0f, 3.0f, 10.0f, M_PI/8);

    // Calculate best Path through using heuristic and maybe some more rendering
    std::vector<glm::mat4> viewpoints = planner.getViewpoints();

    printf("Number of viewpoints: %lu\n", viewpoints.size());

    camera.setDisplayRange(glm::vec2(0.1f, 10000.0f));
    renderer.displayViewpoints(camera, viewpoints, meshes);
    

    return EXIT_SUCCESS;
}

std::vector<AVSCPP::Mesh*> loadMeshes() {
    std::vector<AVSCPP::Mesh*> meshes;

    std::string const modelPath = PROJECT_SOURCE_DIR "/resources/models/aeroplane3.obj";
    AVSCPP::Mesh* modelPoints = new AVSCPP::Mesh(modelPath);
    // glm::mat4 modelMat = glm::yawPitchRoll(0.0f, (float)M_PI/2, 0.0f);
    glm::mat4 modelMat = glm::mat4(1.0);
    modelPoints->setModelMatrix(modelMat);
    meshes.push_back(modelPoints);

    // std::string const floorPath = PROJECT_SOURCE_DIR "/resources/models/plane.obj";
    // AVSCPP::Mesh floorMesh(floorPath);
    // floorMesh.setModelMatrix(glm::mat4(1.0));
    // meshes.push_back(&floorMesh);
    return meshes;
}


void loadShaders(AVSCPP::Renderer &renderer, AVSCPP::CameraControl &camera) {
    // Create and compile our GLSL program from the shaders
    std::string shaderPath = PROJECT_SOURCE_DIR "/AVSCPP/shaders/";
    AVSCPP::Shader* shader = new AVSCPP::Shader();
    shader->attach(shaderPath+"simplevertexshader.vert");
    shader->attach(shaderPath+"simplefragmentshader.frag");
    shader->link();

    AVSCPP::Shader* intshader = new AVSCPP::Shader();
    intshader->attach(shaderPath+"simplevertexshaderint.vert");
    intshader->attach(shaderPath+"simplefragmentshaderint.frag");
    intshader->link();

    AVSCPP::Shader* projshader = new AVSCPP::Shader();
    projshader->attach(shaderPath+"2dbackprojectvert.vert");
    projshader->attach(shaderPath+"2dbackprojectfrag.frag");
    projshader->link();

    renderer.setShaders(shader, projshader, intshader, camera);
}
