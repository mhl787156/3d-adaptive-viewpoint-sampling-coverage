#include "utils.hpp"


void LKHSolver::writeTSPandParamtoFile(std::vector<glm::vec3>& points) {

    int num_points = points.size();

    std::ofstream TSPFile;
    TSPFile.open(TSPfilename);

    TSPFile << "Name : " << name << std::endl;
    TSPFile << "Comment : Lets find some viewpoints" << std::endl;
    TSPFile << "Type : TSP" << std::endl;
    TSPFile << "Dimension : " << num_points << std::endl;
    TSPFile << "EDGE_WEIGHT_TYPE : EUC_3D" << std::endl;
    TSPFile << "NODE_COORD_SELECTION" << std::endl;  

    for(int i = 0; i < num_points; i++) {
        glm::vec3 p = points[i];
        TSPFile << i << " " << p[0] << " " << p[1] << " " << p[2] << std::endl;
    }
    TSPFile.close();
}