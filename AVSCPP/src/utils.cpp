#include "utils.hpp"

#include <sys/stat.h>
#include <set>

using namespace AVSCPP;

std::vector<GLint> LKHSolver::solve(std::vector<glm::vec3> &points, std::vector<float> &weights) {

    writeTSPandParamtoFile(points, weights);

    runLKH();

    std::vector<GLint> traj = readLKHOutput();

    cleanUp();

    return traj;
}

void LKHSolver::writeTSPandParamtoFile(std::vector<glm::vec3>& points, std::vector<float> &weights) {

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
    TSPFile << "EDGE_WEIGHT_TYPE : EXPLICIT" << std::endl;
    TSPFile << "EDGE_WEIGHT_FORMAT : LOWER_ROW" << std::endl;
    TSPFile << "EDGE_WEIGHT_SECTION" << std::endl;  
    int num = 0;
    for(int i = 0; i < num_points; i++) {
        for(int j = 0; j < i; j++) {
            float k = weights[num++];
            TSPFile << int(k * 1000);
            if(j != num_points-1) {
                TSPFile << " ";
            }
        }
        TSPFile << std::endl;
    }

    TSPFile << "EOF" << std::endl;
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
        int idx = std::stoi(line) - 1;
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

float euclideanDistance(glm::mat4& m1, glm::mat4& m2){
    glm::vec3 a = glm::vec3(m1[3]);
    glm::vec3 b = glm::vec3(m2[3]);
    return euclideanDistance(a, b);
}

float euclideanDistance(glm::vec3 &v1, glm::vec3 &v2){
    float x = pow(v1[0] - v2[0], 2.0f);
    float y = pow(v1[1] - v2[1], 2.0f);
    float z = pow(v1[2] - v2[2], 2.0f);
    return sqrt(x + y + z);
}

struct FirstPairComparison {
    bool operator()(const std::pair<float, int>& s1, const std::pair<float, int>& s2) {
       return s1.first < s2.first;
   }
};
