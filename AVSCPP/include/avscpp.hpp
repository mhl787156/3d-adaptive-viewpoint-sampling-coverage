#ifndef AVSCPP_H
#define AVSCPP_H


// System Headers
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/euler_angles.hpp>

// Standard Headers
#include <cstdio>
#include <cstdlib>
#include <utility> 
#include <iostream>
#include <fstream>
#include <limits>
#include <vector>
#include <memory>
#include <set>

// PointCloudLibrary
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


// Octomap
#include <octomap/octomap.h>
#include <octomap/OcTree.h>


// Local Library Headers
#include "render.hpp"
#include "shader.hpp"
#include "mesh.hpp"
#include "controls.hpp"
#include "utils.hpp"

namespace AVSCPP {

class CoveragePlanner {
    public:

        CoveragePlanner() {};
        CoveragePlanner(AVSCPP::Renderer& renderer, AVSCPP::CameraControl& camera, std::vector<AVSCPP::Mesh*> modelMesh);

        // Viewpoint Generation
        void sampleViewpointsNumPoints(std::vector<GLfloat> boundingBox,
                              GLint numX, GLint numY, 
                              GLint numZ, GLfloat resRadians);
        void sampleViewpointsResolution(std::vector<GLfloat> boundingBox,
                              GLfloat resX, GLfloat resY, 
                              GLfloat resZ, GLfloat resRadians);
        
        std::vector<glm::vec3> generatePositions(GLfloat meterResolution);
        std::vector<glm::vec3> generatePositions(GLfloat resX, GLfloat resY, GLfloat resZ);
        std::vector<glm::vec3> generatePositions(std::vector<GLfloat> boundingBox, GLfloat resX, GLfloat resY, GLfloat resZ);
        std::vector<glm::vec3> generatePositions(std::vector<GLfloat> boundingBox, GLint numX, GLint numY, GLint numZ);
        std::vector<float> generateOrientations(GLfloat radiansResolution);

        void addViewpoint(glm::mat4 vp) {viewpoints.push_back(vp);}
        std::vector<glm::mat4>& getViewpoints(){return viewpoints;}
        std::vector<glm::vec3>& getSeenpoints(float res=0.5f);

        std::vector<std::vector<float>> compareAndClusterSeenpointsWithReference(float res=0.2f);
        
        // Path Planning
        void calculateLKHTrajectories(std::vector<glm::vec3> initialPositions);
        void calculateAVSCPPTrajectories(std::vector<glm::vec3> initialPositions);

        void addTrajectoryPoint(int i) {trajectory.push_back(i);}
        std::vector<GLint> getTrajectories(){return trajectory;}

        // Probability Calc
        float computeAccuracy(float deptherror) {return normalizedFocalLength * glm::pow(deptherror, 2) * disparityDeviation;}


        // General Setters and Getters
        void setDepthRange(GLfloat min, GLfloat max) {depthMin=min;depthMax=max;}
        void setDebug(bool d) {debug = d;}
        std::vector<GLfloat> getBoundingBox() {return boundingBox;}
        void setMinResolution(float x, float y, float z) {minResolution = std::vector<GLfloat>{x, y, z};}
    
    private:

        GLfloat depthMin = 1;
        GLfloat depthMax = 5;

        AVSCPP::Renderer *renderer;
        AVSCPP::CameraControl *camera;
        std::vector<AVSCPP::Mesh*> meshes;

        float boundingBoxScaler = 1.5;
        std::vector<GLfloat> boundingBox = {
            std::numeric_limits<float>::max(), std::numeric_limits<float>::min(), // xmin xmax
            std::numeric_limits<float>::max(), std::numeric_limits<float>::min(), // ymin ymax
            std::numeric_limits<float>::max(), std::numeric_limits<float>::min()};// zmin zmax

        std::vector<GLfloat> minResolution{1.0f, 1.0f, 1.0f};


        std::vector<glm::mat4> viewpoints; // Viewpoint sampling outputs
        std::vector<glm::vec3> seenLocations; // Points outputted by the rendering x, y, z, depthFromViewpoint

        float normalizedFocalLength = 2.85e-5;
        float disparityDeviation = 0.5;

        float octreeResolution = 0.3;
        pcl::PointCloud<pcl::PointXYZ>::Ptr objectPointCloud;
        std::unique_ptr<octomap::OcTree> objectOctree;
        double objectOctreeVolume; 
        pcl::PointCloud<pcl::PointXYZ>::Ptr seenPointCloud;
        std::unique_ptr<octomap::OcTree> seenOctree;
        std::shared_ptr<pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ>> octreeChangeDetector;


        double targetCoverage = 0.99;
        double trajectoryCoverage = 0.0;
        float heurisitcDistanceContribFactor = 0.2;
        std::vector<GLint> trajectory; // Indexes into viewpoints
        float calculateEntropyOfViewpoint(glm::mat4& viewpoint, float parentEntropy, bool update);


        bool debug = false;

};

}

#endif