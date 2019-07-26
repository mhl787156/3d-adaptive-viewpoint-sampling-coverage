#ifndef AVSCPP_H
#define AVSCPP_H


// System Headers
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/quaternion.hpp>

// Standard Headers
#include <cstdio>
#include <cstdlib>
#include <utility> 
#include <iostream>
#include <fstream>
#include <limits>
#include <vector>


// Local Library Headers
#include "render.hpp"
#include "shader.hpp"
#include "mesh.hpp"
#include "controls.hpp"

namespace AVSCPP {

class CoveragePlanner {
    public:

        CoveragePlanner() {};
        CoveragePlanner(std::vector<AVSCPP::Mesh*> modelMesh);
        
        std::vector<glm::vec3> generatePositions(GLfloat meterResolution);
        std::vector<glm::vec3> generatePositions(GLfloat resX, GLfloat resY, GLfloat resZ);
        std::vector<float> generateOrientations(GLfloat radiansResolution);

        void addViewpoint(glm::mat4 vp) {viewpoints.push_back(vp);}
        std::vector<glm::mat4>& getViewpoints(){return viewpoints;}
    
    private:

        float boundingBoxScaler = 2.0;
        std::vector<GLfloat> boundingBox = {
            std::numeric_limits<float>::max(), std::numeric_limits<float>::min(), // xmin xmax
            std::numeric_limits<float>::max(), std::numeric_limits<float>::min(), // ymin ymax
            std::numeric_limits<float>::max(), std::numeric_limits<float>::min()};// zmin zmax

        std::vector<glm::mat4> viewpoints; // Viewpoint sampling outputs

};

}

#endif