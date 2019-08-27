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

    objectOctree.reset(new octomap::OcTree(octreeResolution));
    for(pcl::PointXYZ point: objectPointCloud->points) {
        octomap::point3d opoint(point.x, point.y, point.z);
        objectOctree->updateNode(opoint, true);
    }
    objectOctreeVolume = objectOctree -> volume();

    octreeChangeDetector = std::shared_ptr<pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ>>(
                new pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ>(octreeResolution));
}

std::vector<glm::vec3> CoveragePlanner::generatePositions(GLfloat resolution) {
    return generatePositions(resolution, resolution, resolution);
}

std::vector<glm::vec3> CoveragePlanner::generatePositions(GLfloat resX, GLfloat resY, GLfloat resZ) {
    return generatePositions(this->boundingBox, resX, resY, resZ);
}

std::vector<glm::vec3> CoveragePlanner::generatePositions(std::vector<GLfloat> boundingBox, GLfloat resX, GLfloat resY, GLfloat resZ) {
    std::vector<glm::vec3> viewpoint_samples;

    float lowboundX = (boundingBox[0] + boundingBox[1]) / 2.0; 
    float lowboundY = (boundingBox[2] + boundingBox[3]) / 2.0; 
    float lowboundZ = (boundingBox[4] + boundingBox[5]) / 2.0; 

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

std::vector<glm::vec3> CoveragePlanner::generatePositions(std::vector<GLfloat> boundingBox, GLint numX, GLint numY, GLint numZ) {
    std::vector<glm::vec3> viewpoint_samples;

    float resX = (boundingBox[1] - boundingBox[0]) / (float) numX;
    float resY = (boundingBox[3] - boundingBox[2]) / (float) numY;
    float resZ = (boundingBox[5] - boundingBox[4]) / (float) numZ;
    resX = glm::max(resX, minResolution[0]);
    resY = glm::max(resY, minResolution[1]);
    resZ = glm::max(resZ, minResolution[2]);

    for(float x = boundingBox[0]; x <= boundingBox[1]; x+=resX) {
        for(float y = boundingBox[2]; y <= boundingBox[3]; y+=resY) {
            for(float z = boundingBox[4]; z <= boundingBox[5]; z+=resZ) {
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

void CoveragePlanner::sampleViewpointsNumPoints(std::vector<GLfloat> boundingBox,
                                                GLint numX, GLint numY, 
                                                GLint numZ, GLfloat resRadians) {
    std::vector<int> nums{numX, numY, numZ};
    std::vector<float> res{0.0, 0.0, 0.0};
    
    // Reduce number of views until resolution is greater than min resolution
    for(int i = 0; i < res.size(); i++) {
        for(int x = nums[i]; x > 0; x--) {
            res[i] = glm::abs(boundingBox[i+1] - boundingBox[i]) / (float) x;
            if(res[i] > minResolution[i]) {break;}
        }
        if(res[i] < minResolution[i]) {
            // If all too small, set bounding box size of axis to midpoint
            float midpoint = (boundingBox[i] + boundingBox[i+1])/ 2.0;
            boundingBox[i] = midpoint;
            boundingBox[i+1] = midpoint;
            res[i] = minResolution[i];
        }
    }
    printf("Calculated resolution: %f, %f, %f\n", res[0], res[1], res[2]);
    sampleViewpointsResolution(boundingBox, res[0], res[1], res[2], resRadians);
}

void CoveragePlanner::sampleViewpointsResolution(std::vector<GLfloat> boundingBox,
                                                GLfloat resX, GLfloat resY, 
                                                GLfloat resZ, GLfloat resRadians) {
    // viewpoints.clear(); // Clear existing viewpoints

    long unsigned int prevNumberViewpoints = viewpoints.size();

    if(boundingBox.size() != 6) {
        boundingBox = this->boundingBox;
    }

    std::vector<glm::vec3> viewpoint_samples = generatePositions(boundingBox, resX, resY, resZ);
    std::vector<GLfloat> orientation_samples = generateOrientations(M_PI/8);

    // Normal Operataion
    for(glm::vec3 pos: viewpoint_samples) {
        
        GLfloat* pixelLocs;

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
            float numEmpty = 0;
            for(int i = 0; i < renderer->getNumPixels(); i++) {
                float depth = pixelLocs[i*4+3]; // Depth Data
                if(depth > 0 && depth < minDepth){minDepth = depth;}
                if(depth<=0){numBackface++;}
                if(depth>depthMax) {numEmpty++;}
            }

            // If viewpoint is within a depth
            // and isn't entirely backfaces
            // and isn't completely empty
            float percentageBackFace = numBackface / renderer->getNumPixels();
            float percentageEmpty = numEmpty / renderer->getNumPixels();

            if (minDepth < depthMax && minDepth > depthMin 
                && percentageBackFace < 0.5
                && percentageEmpty < 0.9 ) {
                // Then save the viewpoint and insert the points into the pointcloud
                addViewpoint(dronePose);

                for(int i = 0; i < renderer->getNumPixels(); i++) {
                float depth = pixelLocs[i*4 +3];
                if(depth >= depthMin && depth <= depthMax * 2) {
                    float x = pixelLocs[i*4];
                    float y = pixelLocs[i*4+1];
                    float z = pixelLocs[i*4+2];
                    // Add viewpoint to Point cloud for comparison and clustering
                    seenPointCloud->push_back(pcl::PointXYZ(x, y, z));
                }   
            }

            }
        }

        if(debug) {
            printf("-----\n");
        }

    }

    long unsigned int size =  viewpoints.size();
    printf("Number of viewpoints: %lu (%lu new points)\n-----\n", size, size - prevNumberViewpoints);
}

std::vector<std::vector<float>> CoveragePlanner::compareAndClusterSeenpointsWithReference(float res) {
    // Filter Down seenPointCloud into res grid for comparison
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (seenPointCloud);
    vg.setLeafSize (res, res, res); // Filtered to res voxels
    vg.filter (*cloud_filtered);

    // seenPointCloud = cloud_filtered;

    // Compare the points we have seen with reference model
    octreeChangeDetector->setInputCloud(cloud_filtered);
    octreeChangeDetector->addPointsFromInputCloud();

    octreeChangeDetector->switchBuffers();

    octreeChangeDetector->setInputCloud(objectPointCloud); // Object Point Cloud already pre-filtered
    octreeChangeDetector->addPointsFromInputCloud();

    // Output indices of reference pcl
    std::vector<int> missingPoints;

    // Get vector of points from current octree voxels which did not exist in previous buffer
    int numPoints = octreeChangeDetector->getPointIndicesFromNewVoxels(missingPoints);
    octreeChangeDetector->switchBuffers();

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

    if(numPoints < 10) {
        return std::vector<std::vector<float>>();
    }

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
        // std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

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

    std::vector<std::vector<float>> returnBBXs;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        octomap::OcTree tree(octreeResolution);
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
        //     cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
            pcl::PointXYZ point = cloud_filtered->points[*pit];
            octomap::point3d opoint(point.x, point.y, point.z);
            tree.updateNode(opoint, true);
        }

        double xmin, ymin, zmin, xmax, ymax, zmax;
        tree.getMetricMax(xmax, ymax, zmax);
        tree.getMetricMin(xmin, ymin, zmin);

        std::vector<double> bboxd{xmin, xmax, ymin, ymax, zmin, zmax};
        std::vector<float> bbox(bboxd.begin(), bboxd.end());

        // Scale boundingbox
        std::transform(bbox.begin(), bbox.end(), bbox.begin(),
                   std::bind(std::multiplies<float>(), std::placeholders::_1, boundingBoxScaler));


        returnBBXs.push_back(bbox);
    }

    printf("-----\n");
    return returnBBXs;
}

std::vector<glm::vec3>& CoveragePlanner::getSeenpoints(float res){
     // Filter Down seenPointCloud
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (seenPointCloud);
    vg.setLeafSize (res, res, res); // Filtered to 10cm voxels
    vg.filter (*cloud_filtered);

    seenLocations.clear();
    for(pcl::PointXYZ p : cloud_filtered->points) {
        seenLocations.push_back(glm::vec3(p.x, p.y, p.z));
    }

    return seenLocations;
}


void CoveragePlanner::calculateLKHTrajectories(std::vector<glm::vec3> initialPositions) {
    
    printf("Performing Lin-Kernighan heuristic for solving TSP around Viewpoints\n");
    std::vector<glm::vec3> viewpointPositions;
    for(glm::mat4 vp : viewpoints) {
        viewpointPositions.push_back(glm::vec3(vp[3]));
    }
    
    std::vector<GLfloat> weights;
    for(int i = 0; i < viewpoints.size(); i++) {
        glm::vec3 p1 = viewpointPositions[i];
        octomap::point3d p1o(p1.x, p1.y, p1.z);
        for(int j = 0; j < i; j++) {
            glm::vec3 p2 = viewpointPositions[j];
            octomap::point3d p2o(p2.x, p2.y, p2.z);

            if(p1 != p2) {
                float dist = euclideanDistance(p1, p2);
                octomap::point3d dir = p2o - p1o;
                octomap::point3d isect;
                bool intersectsCell = objectOctree->castRay(p1o, dir, isect, true, dist);
                if(intersectsCell) {
                    // If intersects cell, set to large weight
                    weights.push_back(10000.0f);
                } else {
                    weights.push_back(dist);
                }            
            } else {
                weights.push_back(0.0);
            }
        }
    }

    // Perform LKH on viewpoints
    AVSCPP::LKHSolver lkh;
    trajectory = lkh.solve(viewpointPositions, weights);
    printf("Trajectory: ");
    // if(debug) {
        for(GLint k: trajectory){
            printf("%i ", k);
        }
        printf("\n");
    // }
    
    // Find node closest to initial position
    // TODO

    printf("-----\n");
}


float CoveragePlanner::calculateEntropyOfViewpoint(glm::mat4& viewpoint, float parentEntropy, bool update) {
    
    GLfloat* pixelLocs;
    camera->setViewMatrixFromPoseMatrix(viewpoint);
    if(renderer->canRender()) {
        // Render the camera position
        // Calculate pixel locations
        pixelLocs = renderer->getRenderedPositions(*camera, meshes);
    } else {
        exit(0);
    }

    float entropies = parentEntropy;
    for(int i = 0; i < renderer->getNumPixels(); i++) {
        float x = pixelLocs[i*4];
        float y = pixelLocs[i*4+1];
        float z = pixelLocs[i*4+2];
        float depth = pixelLocs[i*4+3];

        if(depth > depthMax || depth < depthMin) {
            continue;
        }

        float accuracy = computeAccuracy(depth);
        float logoddsa = accuracy/(1-accuracy);

        octomap::OcTreeNode* key = seenOctree->search(x, y, z);
        if(key) {
            
            // point already occupied
            float logs = key->getLogOdds();
            float prob =  octomap::probability(logs);//logs/(1+logs);
            float entropyp = -prob * log(prob);

            float newlogs = logs + logoddsa;
            float newprobs = octomap::probability(newlogs); //newlogs / (1 + newlogs);
            float entropya = -newprobs * log(newprobs);

            if(update) {
                seenOctree->updateNode(x, y, z, logoddsa);
            }

            entropies += (entropya - entropyp);
        } else {

            // point not occupied
            float newlogs = octomap::logodds(0.5) + logoddsa; // probability 0.5
            float newprobs = octomap::probability(newlogs); // newlogs / ( 1 + newlogs );
            float entropy = -newprobs * log(newprobs);

            if(update) {
                seenOctree->setNodeValue(x, y, z, newlogs);
            }

            entropies += entropy;
        }

    }

    return entropies;
}

void CoveragePlanner::calculateAVSCPPTrajectories(std::vector<glm::vec3> initialPositions) {
    printf("Performing AVSCPP to solve TSP around viewpoints\n");

    // initialise octree
    seenOctree.reset(new octomap::OcTree(octreeResolution));

    printf("Here!\n");

    // Generate Neighbourhoods using minimum spanning tree
    // std::vector<std::vector<int>> mst = minimumSpanningTree(viewpoints);

    // int j = 0;
    // for(auto vec: mst) {
    //     printf("node %i: ", j);
    //     for(int v : vec) {
    //         printf("%i ", v);
    //     }
    //     printf("\n");
    //     j++;
    // }

    float currentEntropy = 0.0;
    int currentS = 0; //(int) viewpoints.size()/2.0;

    std::set<int> closedList; // already visited

    trajectory.push_back(currentS);
    closedList.insert(currentS);

    while(trajectoryCoverage < targetCoverage) {
        
        printf("currentS: %i (%f, %f, %f)\n", currentS, 
                viewpoints[currentS][3][0], viewpoints[currentS][3][1], viewpoints[currentS][3][2]);
        printf("Coverage: %f\n", trajectoryCoverage);
        
        closedList.insert(currentS);

        float radii = 2.0;

        std::vector<int> inRadii;

        // Pick sbest from neighbours
        int sbest = -1;
        float bestR = -1.0f;
        float bestEntropy = -1.0f;

        std::vector<int> onSameLoc;
        std::set<int> alreadyConsidered;

        glm::mat4 parent = viewpoints[currentS];
        glm::vec3 parentloc = glm::vec3(parent[3]);
        octomap::point3d parentlocp3d(parentloc.x, parentloc.y, parentloc.z);
        for(radii; radii < 20.0; radii+=2.0) {
            // Take current vp and look for those within a radius
            for(int r = 0; r < viewpoints.size(); r++) {
                float dist = euclideanDistance(viewpoints[r], viewpoints[currentS]);
                if( dist < radii
                    && closedList.find(r) == closedList.end() // not in closed list
                    && alreadyConsidered.find(r) == alreadyConsidered.end()) { // not in already considered
                                        
                    // In Radius!
                    alreadyConsidered.insert(r);

                    glm::mat4 current = viewpoints[r];
                    glm::vec3 currentloc = glm::vec3(current[3]);
                    octomap::point3d currentlocp3d(currentloc.x, currentloc.y, currentloc.z);
                    
                    // float dist = euclideanDistance(parentloc, currentloc);

                    // Not on the same point
                    if(parentloc != currentloc) {
                        // Test for intersection
                        // octomap::KeyRay ray;
                        // bool rayfound = objectOctree->computeRayKeys(parentlocp3d, currentlocp3d, ray);
                        // bool intersects = false;
                        // for(octomap::OcTreeKey k: ray) {
                        //     if(objectOctree->search(k) != NULL) {intersects = true; break;}
                        // }
                        // if(intersects){
                        //     continue;
                        // }

                        octomap::point3d dir = currentlocp3d - parentlocp3d;
                        octomap::point3d isect;
                        bool intersectsCell = objectOctree->castRay(parentlocp3d, dir, isect, true, dist);
                        if(intersectsCell) {
                            continue;
                        }

                    } else {
                        onSameLoc.push_back(r);
                    }
                    
                    // Calculate entropy and heuristic value R
                    float entropy = calculateEntropyOfViewpoint(viewpoints[r], currentEntropy, false);
                    float R = entropy * exp(-heurisitcDistanceContribFactor * dist);
                    if(R >= bestR) {
                        sbest = r; bestR = R; bestEntropy = entropy;
                    }


                }
            }

            if(sbest == -1) {
                printf("Cannot find any point, expand radius\n");
            } else {
                break;
            }
        }

        // If no next point is found...
        if(sbest == -1) {
            printf("Cannot path to anymore points, all intersecting\n");
            break;
        }

        // With best R apply coverage
        currentEntropy = calculateEntropyOfViewpoint(viewpoints[sbest], currentEntropy, true);
        printf("Sbest(%i) Current Entropy, R: %f, %f\n", sbest, currentEntropy, bestR);

        // Calculate new coverage percentage
        double seenOctreeVolume = seenOctree->volume();
        printf("Seen Volume %f\n", seenOctreeVolume);
        trajectoryCoverage = seenOctreeVolume / objectOctreeVolume;

        // Insert new trajectory point
        trajectory.push_back(sbest);

        // Insert all the other points on the same point and remove from consideration
        for(int s: onSameLoc) {
            trajectory.push_back(s);
            closedList.insert(s);
        }

        // set current node to best node
        currentS = sbest;

        // Check if all viewpoints on trajectory
        // if(trajectory.size() >= viewpoints.size()){
        //     printf("All Viewpoints on Trajectory\n");
        //     break;
        // }
    }

    printf("Trajectory: ");
    for(GLint i: trajectory) {
        printf("%i, ", i);
    }
    printf("\n");
}

std::vector<float> CoveragePlanner::calculateTrajectoriesLengths() {

    std::vector<float> lengths;
    for(std::vector<GLint> traj: trajectories) {
        float len = 0;
        glm::vec3 prev_point = glm::vec3(viewpoints[traj[0]][3]);
        for(int i = 1; i < traj.size(); i++) {
            glm::vec3 curr_point = glm::vec3(viewpoints[traj[i]][3]);
            len += euclideanDistance(prev_point, curr_point);
            prev_point = curr_point;
        }
        lengths.push_back(len);
    }
    return lengths;
}

// TODO
void CoveragePlanner::allocateMultiAgentTrajectories(int num_drones) {

    std::vector<GLint> trajs;
    int i = 1;
    int thresh = ((int) (trajectory.size() / ((float) num_drones))) + 1;
    printf("Allocating Trajectories (%i/%lu):\n%i: ", thresh, trajectory.size(), i);
    for(int j = 0; j < trajectory.size(); j++) {
        if(j >= i * thresh) {
            printf(", Length (%lu)", trajs.size());
            trajectories.push_back(trajs);
            trajs = std::vector<GLint>();
            i++;
            printf("\n%i: ", i);
        }

        trajs.push_back(trajectory[j]);
        printf("%i ", trajectory[j]);
    }
    trajectories.push_back(trajs);
    printf(", Length (%lu)\n", trajs.size());

    std::vector<float> lengths = calculateTrajectoriesLengths();
    printf("Trajectory lengths: ");
    for(float l: lengths) {
        printf("%f, ", l);
    }
    printf("\n");

    printf("-----\n");
}