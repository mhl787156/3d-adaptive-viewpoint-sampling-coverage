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

class LKHSolver {

    public:

        std::vector<glm::vec3>& solve(std::vector<glm::vec3> &points);

        std::string name = "viewpoints";
        std::string LKHDir = PROJECT_SOURCE_DIR "/external/LKH";
        std::string LKHExe = "LKH";
        std::string tmpLKHDir = PROJECT_SOURCE_DIR "/tmp/";
        std::string TSPfilename = tmpLKHDir + "viewpoints.tsp";
        std::string Paramfilename = tmpLKHDir + "viewpoints.par";
        std::string Outputfilename = tmpLKHDir + "viewpoints.output";

    private:
        void writeTSPandParamtoFile(std::vector<glm::vec3> &points);


};


#endif