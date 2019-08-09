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

    // Initialise PointCloud
    objectPointCloud = modelMesh[0]->getPointCloud();
    seenPointCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    // seenPointCloud->width = objectPointCloud->width * 2;
    // seenPointCloud->height = 1;
    // seenPointCloud->points.resize(seenPointCloud->width * seenPointCloud->height);


    // pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
    // viewer.showCloud(objectPointCloud);
    // while(!viewer.wasStopped()) {}
    // viewer.~CloudViewer();


    octree = std::shared_ptr<pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ>>(
                new pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ>(octreeResolution));
    // octree->setInputCloud(objectPointCloud);
    // octree->addPointsFromInputCloud();
    // octree->switchBuffers();
    // octree->setInputCloud(seenPointCloud);
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
                    float x = pixelLocs[i*4];
                    float y = pixelLocs[i*4+1];
                    float z = pixelLocs[i*4+2];
                    seenLocations.push_back(glm::vec3(x, y, z));
                    seenPointCloud->push_back(pcl::PointXYZ(x, y, z));
                    // octree->addPointToCloud(pcl::PointXYZ(x, y, z), seenPointCloud);
                }   
            }
        }

        if(debug) {
            printf("-----\n");
        }

    }
    printf("Number of viewpoints: %lu\n-----\n", viewpoints.size());
}

void CoveragePlanner::compareSeenpointsWithReference() {
    // Filter Down seenPointCloud
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (seenPointCloud);
    vg.setLeafSize (0.2f, 0.2f, 0.2f); // Filtered to 10cm voxels
    vg.filter (*cloud_filtered);

    // Compare the points we have seen with reference model
    octree->setInputCloud(cloud_filtered);
    octree->addPointsFromInputCloud();

    octree->switchBuffers();

    octree->setInputCloud(objectPointCloud);
    octree->addPointsFromInputCloud();

    // Output indices of reference pcl
    std::vector<int> missingPoints;

    // Get vector of points from current octree voxels which did not exist in previous buffer
    int numPoints = octree->getPointIndicesFromNewVoxels(missingPoints);

    // Create new pointcloud to hold output
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmppcl(new pcl::PointCloud<pcl::PointXYZ>);
    tmppcl->width = missingPoints.size();
    tmppcl->height = 1;
    tmppcl->points.resize(tmppcl->width);

    for(int i = 0; i < missingPoints.size(); i++) {
        auto point = objectPointCloud->points[missingPoints[i]];
        tmppcl->points[i].x = point.x;
        tmppcl->points[i].y = point.y;
        tmppcl->points[i].z = point.z;
    }

    printf("%i points were identified as missing out of %lu reference\n", numPoints, objectPointCloud->size());

    // pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
    // viewer.showCloud(tmppcl);
    // while(!viewer.wasStopped()) {}
    // viewer.~CloudViewer();


    ///////
    /////// Begin Clustering
    ///////
    cloud_filtered = tmppcl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);
    int i=0, nr_points = (int) cloud_filtered->points.size ();
    while (cloud_filtered->points.size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *cloud_filtered = *cloud_f;
    }
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (1.0f); // 1m
    ec.setMinClusterSize (50);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
        }
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::stringstream ss;
        ss << "cloud_cluster_" << j << ".pcd";
        writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        j++;
    }

    printf("-----\n");
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

