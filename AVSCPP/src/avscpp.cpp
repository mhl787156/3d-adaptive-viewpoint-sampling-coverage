#include "avscpp.hpp"
#include <iostream> // std::cout 
#include <functional> // std::multiplies 
#include <algorithm> // std::transform 
#include <math.h>

using namespace AVSCPP;

CoveragePlanner::CoveragePlanner(AVSCPP::Renderer &renderer, AVSCPP::CameraControl &camera, std::vector<AVSCPP::Mesh*> modelMesh) {

    this->renderer = &renderer;
    this->camera = &camera;
    this->meshes = modelMesh;

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
    return generatePositions(resolution, resolution, resolution);
}

std::vector<glm::vec3> CoveragePlanner::generatePositions(GLfloat resX, GLfloat resY, GLfloat resZ) {
    return generatePositions(this->boundingBox, resX, resY, resZ);
}

std::vector<glm::vec3> CoveragePlanner::generatePositions(std::vector<GLfloat> boundingBox, GLfloat resX, GLfloat resY, GLfloat resZ) {
    std::vector<glm::vec3> viewpoint_samples;

    float lowboundX = 0; 
    float lowboundY = 0; 
    float lowboundZ = 0; 

    while(lowboundX > boundingBox[0] || lowboundY > boundingBox[2] || lowboundZ > boundingBox[4] ) {
        if(lowboundX > boundingBox[0]){lowboundX -= resX;}
        if(lowboundY > boundingBox[2]){lowboundY -= resY;}
        if(lowboundZ > boundingBox[4]){lowboundZ -= resZ;}
    }

    for(float x = lowboundX + resX; x <= boundingBox[1]; x+=resX) {
        for(float y = lowboundY + resY; y <= boundingBox[3]; y+=resY) {
            for(float z = lowboundZ + resZ; z <= boundingBox[5]; z+=resZ) {
                viewpoint_samples.push_back(glm::vec3(x, y, z));
            }
        }
    }

    return viewpoint_samples;
}

std::vector<float> CoveragePlanner::generateOrientations(GLfloat resolution) {
    std::vector<float> orientation_samples;
    for(float yaw = 0.0; yaw <= 2*M_PI; yaw+=resolution) {
        orientation_samples.push_back(yaw);
    }
    return orientation_samples;
}

void CoveragePlanner::sampleViewpoints(std::vector<GLfloat> boundingBox,
                                                GLfloat resX, GLfloat resY, 
                                                GLfloat resZ, GLfloat resRadians) {
    viewpoints.clear(); // Clear existing viewpoints

    if(boundingBox.size() != 6) {
        boundingBox = this->boundingBox;
    }

    std::vector<glm::vec3> viewpoint_samples = generatePositions(boundingBox, resX, resY, resZ);
    std::vector<GLfloat> orientation_samples = generateOrientations(M_PI/8);

    // Normal Operataion
    for(glm::vec3 pos: viewpoint_samples) {
        
        GLfloat* pixelLocs;

        // For each camera location
        float minYawDepth = 100000.0;
        glm::mat4 bestyawpose = glm::mat4(1.0);

        for(float yaw: orientation_samples) {
            // For each discretised camera angle
            glm::mat4 dronePose = glm::yawPitchRoll(yaw, 0.0f, 0.0f);
            dronePose[3] = glm::vec4(pos, 1.0);
            camera->setViewMatrixFromPoseMatrix(dronePose);

            if(renderer->canRender()) {
                // Render the camera position
                // Calculate pixel locations
                pixelLocs = renderer->getRenderedPositions(*camera, meshes);
            } else {
                break;
            }
            
            // Store minimum depth
            float minDepth = 10000000.0f;
            float numBackface = 0;
            for(int i = 0; i < renderer->getNumPixels(); i++) {
                float depth = pixelLocs[i*4+3]; // Depth Data
                if(depth > 0 && depth < minDepth){minDepth = depth;}
                if(depth<=0){numBackface++;}
            }

            float percentageBackFace = numBackface / renderer->getNumPixels();

            if (minDepth < minYawDepth && minDepth > depthMin && percentageBackFace < 0.5) {
                minYawDepth = minDepth;
                bestyawpose = dronePose;
            }
        }
        
        // If all greater than some threshold, remove camera location, continue
        // This does distance and collision filtering
        if(minYawDepth < depthMax) {
            addViewpoint(bestyawpose);
            camera->setViewMatrixFromPoseMatrix(bestyawpose);
            
            // Record points
            pixelLocs = renderer->getRenderedPositions(*camera, meshes);
            for(int i = 0; i < renderer->getNumPixels(); i++) {
                float depth = pixelLocs[i*4 +3];
                if(depth >= depthMin && depth <= depthMax * 2) {
                    seenLocations.push_back(glm::vec3(pixelLocs[i*4], pixelLocs[i*4+1], pixelLocs[i*4+2]));
                }
            }
        }

        if(debug) {
            printf("-----\n");
        }

    }
    printf("Number of viewpoints: %lu\n-----\n", viewpoints.size());
}

void CoveragePlanner::calculateLKHTrajectories(std::vector<glm::vec3> initialPositions) {
    
    printf("Performing Lin-Kernighan heuristic for solving TSP around Viewpoints\n");
    std::vector<glm::vec3> viewpointPositions;
    for(glm::mat4 vp : viewpoints) {
        viewpointPositions.push_back(glm::vec3(vp[3]));
    }

    // Perform LKH on viewpoints
    AVSCPP::LKHSolver lkh;
    trajectory = lkh.solve(viewpointPositions);

    if(debug) {
        for(GLint k: trajectory){
            printf("%i ", k);
        }
        printf("\n");
    }
    
    

    // Find node closest to initial position
    // TODO

    printf("-----\n");
}

