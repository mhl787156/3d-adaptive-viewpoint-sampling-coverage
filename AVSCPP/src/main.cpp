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
int iterations = 1;
bool runLKH = false;
int numViews = 4;
int numDrones = 2;
float resolution = 0.0;
bool forceSquare = false;
std::string objectfilename="aeroplane3";

// Function declarations
std::vector<AVSCPP::Mesh*> loadMeshes();
void loadShaders(AVSCPP::Renderer &renderer, AVSCPP::CameraControl &camera);
std::vector<float> forceSquareBBOX(std::vector<float> boundingbox);

int main(int argc, char * argv[]) {

    for (int i = 1; i < argc; i++) {
        if(std::string(argv[i]) == "-d") {setDebug = true; continue;}
        if(std::string(argv[i]) == "-s") {renderToScreen = true; continue;}
        if(std::string(argv[i]) == "-rt") {renderTesting = true; continue;}
        if(std::string(argv[i]) == "-w") {waitMillis = std::stoi(std::string(argv[++i])); continue;}
        if(std::string(argv[i]) == "-it") {iterations = std::stoi(std::string(argv[++i])); continue;}
        if(std::string(argv[i]) == "-res") {resolution = std::stof(std::string(argv[++i])); continue;}
        if(std::string(argv[i]) == "-lkh") {runLKH = true; continue;}
        if(std::string(argv[i]) == "-fsquare") {forceSquare = true; continue;}
        if(std::string(argv[i]) == "-nv") {numViews = std::stoi(std::string(argv[++i])); continue;}
        if(std::string(argv[i]) == "-n") {numDrones = std::stoi(std::string(argv[++i])); continue;}
        if(std::string(argv[i]) == "-filename") {objectfilename = std::string(argv[++i]); continue;}
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
    planner.setMinResolution(1.0, 1.0, 1.0);
    // planner.setMinResolution(0.2, 0.2, 0.2);

    std::vector<float> bbox = planner.getBoundingBox();
    // if(forceSquare) {bbox = forceSquareBBOX(bbox);}
    if(resolution > 0) {
        planner.sampleViewpointsResolution(bbox, resolution, resolution, resolution, M_PI/8);    
    } else {
        planner.sampleViewpointsNumPoints(bbox, numViews, numViews, numViews, M_PI/8);    
    }
    
    // planner.sampleViewpointsNumPoints(bbox, 4, 3, 4, M_PI/8);
    // planner.sampleViewpointsResolution(bbox, 2.5, 2.5, 2.5, M_PI/8);

    std::vector<std::vector<float>> boundingBoxes;
    boundingBoxes.push_back(bbox);

    for(int it = 0; it < iterations; it++) {
        printf("######## Iteration %i #######\n", it+1);

        boundingBoxes = planner.compareAndClusterSeenpointsWithReference(0.1f);
        for(auto v: boundingBoxes) {
            printf("New bounding box: ");
            for(float f: v) {
                printf("%f ", f);
            }
            printf("\n");
        }

        if(boundingBoxes.size() == 0) {
            printf("No further clusterings found\n");
            break;
        }

        for(std::vector<float> bbox: boundingBoxes) {
            // if(forceSquare) {bbox = forceSquareBBOX(bbox);}
            // Planner renders a set of viewpoints using a default resolution
            // passing an empty vector will cause the full Mesh bounding box to be used.
            planner.sampleViewpointsNumPoints(bbox, numViews, numViews, numViews, M_PI/8);
        }

        boundingBoxes.clear();
    }
    printf("######## Iterations Complete #######\n");



    std::vector<glm::vec3> initialPos;
    // initialPos.push_back(glm::vec3(10.0, 10.0, 10.0));
    initialPos.push_back(glm::vec3(2.0, 2.0, 2.0));
    if(runLKH) {
        planner.calculateLKHTrajectories(initialPos);
    } else {
        planner.calculateAVSCPPTrajectories(initialPos);
    }


    planner.allocateMultiAgentTrajectories(numDrones);

    // Calculate best Path through using heuristic and maybe some more rendering
    std::vector<glm::mat4> viewpoints = planner.getViewpoints();
    std::vector<glm::vec3> seenpoints = planner.getSeenpoints(0.1f);
    std::vector<std::vector<GLint>> trajectories = planner.getTrajectories();

    std::stringstream vpfilename;
    vpfilename << objectfilename << "_r" << resolution << "_n" << numViews;
    std::string vpfname = vpfilename.str();    
    saveViewpointsToFile(vpfname, viewpoints);

    printf("Displaying Viewpoints");
    camera.setDisplayRange(glm::vec2(0.1f, 10000.0f));
    renderer.displayViewpoints(camera, viewpoints, trajectories, seenpoints, meshes);    

    return EXIT_SUCCESS;
}

std::vector<AVSCPP::Mesh*> loadMeshes() {
    std::vector<AVSCPP::Mesh*> meshes;

    std::string const modelPath = PROJECT_SOURCE_DIR "/resources/models/" + objectfilename;
    AVSCPP::Mesh* modelPoints = new AVSCPP::Mesh(modelPath);
    // glm::mat4 modelMat = glm::yawPitchRoll(0.0f, (float)M_PI, 0.0f);
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

std::vector<float> forceSquareBBOX(std::vector<float> boundingBox) {
    int best_i = 0;
    GLfloat max_width = 0;
    for(int i=0; i <3; i++){
        GLfloat axiswidth = boundingBox[i*2+1] - boundingBox[i*2];
        if(axiswidth > max_width){max_width = axiswidth; best_i =i;}
    }
    GLfloat bestmin = boundingBox[best_i*2];
    GLfloat bestmax = boundingBox[best_i*2+1];
    for(int i=0; i <3; i++){
        boundingBox[i*2+1] = bestmax;
        boundingBox[i*2] = bestmin;
    }
    return boundingBox;
}
