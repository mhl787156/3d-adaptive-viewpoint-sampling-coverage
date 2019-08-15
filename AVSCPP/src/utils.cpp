#include "utils.hpp"

#include <sys/stat.h>
#include <set>

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

int minKey(std::vector<int> &keys, std::vector<bool> &mstSet) {
    int min = INT_MAX;
    int min_index = 0;
    for(int v = 0; v < mstSet.size(); v++) {
        if(!mstSet[v] && keys[v] < min){
            min = keys[v]; min_index = v;
        }
    }
    return min_index;
}

std::vector<std::vector<int>> minimumSpanningTree(std::vector<glm::mat4> &vps) {

    std::vector<int> mstrep(vps.size());
    std::vector<std::set<int>> mst(vps.size());

    std::vector<int> keys(vps.size());

    std::vector<bool> mstSet(vps.size());

    for(int i = 0; i < keys.size(); i++) {
        keys[i] = INT_MAX; mstSet[i] = false;
        mst[i] = std::set<int>();
    }

    keys[0] = 0;

    for(int count = 0; count < vps.size() - 1; count++) {
        
        int u = minKey(keys, mstSet);

        mstSet[u] = true;

        for(int v = 0; v < keys.size(); v++) {
            if(u == v) {continue;}
            glm::vec3 u_vps = glm::vec3(vps[u][3]);
            glm::vec3 v_vps = glm::vec3(vps[v][3]);
            glm::vec3 diff = glm::abs(u_vps - v_vps);
            float graphUV = sqrt(diff[0] + diff[1] + diff[2]);


            if(graphUV == 0) {
                mst[u].insert(v);
                mst[v].insert(u);
                // mstSet[v] = true;
                continue;
            }

            if(!mstSet[v] && graphUV < keys[v]) {
                mstrep[v] = u;
                keys[v] = graphUV;
            }
        }
    }

    std::vector<std::vector<int>> ret(vps.size());
    for(int i = 0; i < mstrep.size(); i++) {
        ret[i] = std::vector<int>(mst[i].begin(), mst[i].end());
        ret[i].push_back(mstrep[i]);
    }

    return ret;
} 