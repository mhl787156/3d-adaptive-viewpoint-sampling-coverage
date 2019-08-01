#include "utils.hpp"

#include <sys/stat.h>

using namespace AVSCPP;

std::vector<GLint> LKHSolver::solve(std::vector<glm::vec3> &points) {

    writeTSPandParamtoFile(points);

    runLKH();

    std::vector<GLint> traj = readLKHOutput();

    cleanUp();

    return traj;
}

void LKHSolver::writeTSPandParamtoFile(std::vector<glm::vec3>& points) {

    int num_points = points.size();

    // Ensure tmp directory is created
    struct stat sb;
    if (!(stat(tmpLKHDir.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode)))
    {
        int status = mkdir(tmpLKHDir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        if (-1 == status)
        {
            printf("Error creating directory!: %s\n", tmpLKHDir.c_str());
            exit(1);
        }
    }

    std::ofstream TSPFile;
    TSPFile.open(TSPfilename);

    if(!TSPFile.is_open()) {
        printf("LKH TSPFIle did not open! %s\n", TSPfilename.c_str());
        exit(1);
    }

    TSPFile << "Name : " << name << std::endl;
    TSPFile << "Comment : Lets find some viewpoints" << std::endl;
    TSPFile << "Type : TSP" << std::endl;
    TSPFile << "Dimension : " << num_points << std::endl;
    TSPFile << "EDGE_WEIGHT_TYPE : EUC_3D" << std::endl;
    TSPFile << "NODE_COORD_TYPE : THREED_COORDS" << std::endl;
    TSPFile << "NODE_COORD_SECTION" << std::endl;  

    for(int i = 0; i < num_points; i++) {
        glm::vec3 p = points[i];
        TSPFile << i+1 << " " << p[0] << " " << p[1] << " " << p[2] << std::endl;
    }
    TSPFile.close();
    printf("LKH TSPFile Written\n");

    std::ofstream ParamFile;
    ParamFile.open(Paramfilename);

    if(!ParamFile.is_open()) {
        printf("LKH Paramfile did not open! %s\n", Paramfilename.c_str());
        exit(1);
    }

    ParamFile << "PROBLEM_FILE = " << TSPfilename << std::endl;
    ParamFile << "TOUR_FILE = " << Outputfilename << std::endl;

    ParamFile.close();
    printf("LKH ParamFile Written\n");
}

void LKHSolver::runLKH() {

    std::string exe = LKHDir + LKHExe + " " + Paramfilename;

    FILE *handle = popen( exe.c_str(), "r");

    if (handle == NULL) {
        printf("LKH executable unable to be opened: %s\n", exe.c_str());
        exit(1);
    } else {
        printf("LKH running: %s\n", exe.c_str());
    }

    char buf[512];
    size_t readn;
    while ((readn = fread(buf, 1, sizeof(buf), handle)) > 0) {
        if(debug){fwrite(buf, 1, readn, stdout);}
    }
    pclose(handle);
}

std::vector<GLint> LKHSolver::readLKHOutput() {
    // Parse output file
    std::vector<GLint> traj_idxs;
    
    std::ifstream outputfile(Outputfilename);
    std::string line;

    while(std::getline(outputfile, line)) {
        if(line == "TOUR_SECTION") {
            break;
        }
        if(line.find("Length") != std::string::npos) {
            std::istringstream iss(line);
            std::string a, b, c, d;
            iss >> a >> b >> c >> d >> tourLength;
        }
    }

    while(std::getline(outputfile, line)) {
        if(line == "-1") {
            break;
        }
        int idx = std::stoi(line);
        traj_idxs.push_back(idx);
    }
    
    return traj_idxs;
}

void LKHSolver::cleanUp() {
    // Remove files

    std::string command = "rm -r " + tmpLKHDir;
    std::system(command.c_str());
}
