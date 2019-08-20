#ifndef UTILS_H
#define UTILS_H

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
#include <sstream>
#include <string>

namespace AVSCPP {

class LKHSolver {
    public:
        std::vector<GLint> solve(std::vector<glm::vec3> &points);
        void setDebug(bool d) {debug = d;}

        float getTourLength() {return tourLength;}

        std::string name = "viewpoints";
        std::string LKHDir = PROJECT_SOURCE_DIR "/external/LKH/";
        std::string LKHExe = "LKH";
        std::string tmpLKHDir = PROJECT_SOURCE_DIR "/tmp_lkh/";
        std::string TSPfilename = tmpLKHDir + "viewpoints.tsp";
        std::string Paramfilename = tmpLKHDir + "viewpoints.par";
        std::string Outputfilename = tmpLKHDir + "viewpoints.output";

    private:
        void writeTSPandParamtoFile(std::vector<glm::vec3> &points);
        void runLKH();
        std::vector<GLint> readLKHOutput();
        void cleanUp();

        float tourLength = -1;

        bool debug = false;
};

};

std::vector<std::vector<int>> minimumSpanningTree(std::vector<glm::mat4> &vps);

float euclideanDistance(glm::mat4& m1, glm::mat4& m2);
float euclideanDistance(glm::vec3 &v1, glm::vec3 &v2);

#endif