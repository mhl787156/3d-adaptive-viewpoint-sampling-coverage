#include "avscpp.hpp"
#include <iostream> // std::cout 
#include <functional> // std::multiplies 
#include <algorithm> // std::transform 
#include <math.h>

using namespace AVSCPP;

CoveragePlanner::CoveragePlanner(std::vector<AVSCPP::Mesh*> modelMesh) {

    // Get bounding box for all meshes 
    for(AVSCPP::Mesh* m : modelMesh) {
        std::vector<GLfloat> sb = m->getBoundingBox(boundingBox);
        if(sb[0] < boundingBox[0]){boundingBox[0] = sb[0];} //xmin
        if(sb[1] > boundingBox[1]){boundingBox[1] = sb[1];} //xmax
        if(sb[2] < boundingBox[2]){boundingBox[2] = sb[2];} //ymin
        if(sb[3] > boundingBox[3]){boundingBox[3] = sb[3];} //ymin
        if(sb[4] < boundingBox[4]){boundingBox[4] = sb[4];} //zmin
        if(sb[5] > boundingBox[5]){boundingBox[5] = sb[5];} //zmax
    }
    
    // Scale boundingbox
    std::transform(boundingBox.begin(), boundingBox.end(), boundingBox.begin(),
                   std::bind(std::multiplies<float>(), std::placeholders::_1, boundingBoxScaler));

    printf("Mesh Boundary: ");
    for(float b :boundingBox) {printf("%f ", b);} printf("\n");
}

std::vector<glm::vec3> CoveragePlanner::generatePositions(GLfloat resolution) {
    std::vector<glm::vec3> viewpoint_samples;

    for(float x = boundingBox[0]; x <= boundingBox[1]; x+=resolution) {
        for(float y = boundingBox[2]; y <= boundingBox[3]; y+=resolution) {
            for(float z = boundingBox[4]; z <= boundingBox[5]; z+=resolution) {
                viewpoint_samples.push_back(glm::vec3(x, y, z));
            }
        }
    }

    return viewpoint_samples;
}

std::vector<float> CoveragePlanner::generateOrientations(GLfloat resolution) {
    std::vector<float> orientation_samples;
    for(float yaw = 0; yaw <= 2*M_PI; yaw+=resolution) {
        orientation_samples.push_back(yaw);
    }
    return orientation_samples;
}

