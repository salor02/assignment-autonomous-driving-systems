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
#include <pcl/filters/crop_box.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <chrono>
#include <unordered_set>
#include <boost/filesystem.hpp>
#include <unistd.h>
#include "../include/Renderer.hpp"
#include "../include/tree_utilities.hpp"

namespace fs = boost::filesystem;

using namespace lidar_obstacle_detection;

#define USE_PCL_LIBRARY

typedef std::unordered_set<int> my_visited_set_t;

// downsapling parameters
const static float LEAF_X = 0.1;
const static float LEAF_Y = 0.1;
const static float LEAF_Z = 0.1;

// RANSAC segmentation parameters
const static double PLANE_DIST_TRESHOLD = 0.1;
const static double SEGMENTATION_TRESHOLD = 0.6;

// clustering parameters
const static double CLUSTER_TOLERANCE = 0.2;
const static int MIN_CLUSTER_SIZE = 100;
const static int MAX_CLUSTER_SIZE = 25000;


// sets up the custom kdtree using the point cloud
void setupKdtree(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, my_pcl::KdTree* tree, int dimension){
    // insert point cloud points into tree
    for (int i = 0; i < cloud->size(); ++i){
        tree->insert({cloud->at(i).x, cloud->at(i).y, cloud->at(i).z}, i);
    }
}

/*
    This function computes the nearest neighbors and builds the clusters.
    This is a recursive function that keeps adding points to the cluster searching them in the previously built
    KD-tree in a range equal to distanceTol distance. This function's stopping condition is represented by the impossibility to add
    a new point in the cluster, either because the max cluster size has been reached or because no further neighbors has been found.
        - Input:
            + cloud: Point cloud to be explored
            + target_ndx: i-th point to visit
            + tree: kd tree for searching neighbors
            + distanceTol: Distance tolerance to build the clusters 
            + visited: Visited points --> typedef std::unordered_set<int> my_visited_set_t;
            + cluster: Here we add points that will represent the cluster
            + max: Max cluster size
        - Output:
            + visited: already visited points
            + cluster: at the end of this function we will have one cluster
*/
void proximity(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int target_ndx, my_pcl::KdTree* tree, float distanceTol, my_visited_set_t& visited, std::vector<int>& cluster, int max){
	if (cluster.size() < max){
        cluster.push_back(target_ndx);
        visited.insert(target_ndx);

        std::vector<float> point {cloud->at(target_ndx).x, cloud->at(target_ndx).y, cloud->at(target_ndx).z};
    
        // get all neighboring indices of point
        std::vector<int> neighborNdxs = tree->search(point, distanceTol);

        for (int neighborNdx : neighborNdxs){
            // if point was not visited
            if (visited.find(neighborNdx) == visited.end()){
                proximity(cloud, neighborNdx, tree, distanceTol, visited, cluster, max);
            }

            if (cluster.size() >= max){
                return;
            }
        }
    }
}

/*
    This function builds the clusters following a euclidean clustering approach
        - Input:
            + cloud: Point cloud to be explored
            + tree: kd tree for searching neighbors
            + distanceTol: Distance tolerance to build the clusters 
            + setMinClusterSize: Minimum cluster size
            + setMaxClusterSize: Max cluster size
        - Output:
            + cluster: at the end of this function we will have a set of clusters
*/
std::vector<pcl::PointIndices> euclideanCluster(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, my_pcl::KdTree* tree, float distanceTol, int MinClusterSize, int MaxClusterSize)
{
    // already visited points
	my_visited_set_t visited{};   
    
    // vector of PointIndices that will contain all the clusters, each PointIndices object is essentially a vector of int values
	std::vector<pcl::PointIndices> clusters; 
    
    // vector of int values representing the indices of points included in the cluster by the proximity function
    std::vector<int> cluster; 
    
    /*
        The following lines do:
            1. iterates over every point of the cloud
            2. if the point has not been visited yet it means that it's possibile to build a new cluster starting from that point,
               then the proximity function is launched.
            3. if the computed cluster is large enough then it is included in the cluster list
    */
    for(int i = 0; i < cloud->size(); ++i){
        if(visited.find(i) == visited.end()){
            proximity(cloud, i, tree, distanceTol, visited, cluster, MaxClusterSize);
            if(cluster.size() >= MinClusterSize){
                pcl::PointIndices cluster_indices;
                cluster_indices.indices = cluster;
                clusters.push_back(cluster_indices);
            }
            cluster.clear();
        }
    }
	return clusters;	
}

// executes voxel filtering on the original cloud and builds the filtered cloud
void downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_downsampled){
    pcl::VoxelGrid<pcl::PointXYZ> downsampler;
    downsampler.setInputCloud(cloud);
    downsampler.setLeafSize(LEAF_X, LEAF_Y, LEAF_Z);
    downsampler.filter(*cloud_downsampled);
}

// crops the points that are far away from ego vehicle
void crop(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_cropped){
    pcl::CropBox<pcl::PointXYZ> cb(true);
    cb.setInputCloud(cloud_cropped);
    cb.setMin(Eigen::Vector4f (-20, -6, -2, 1));
    cb.setMax(Eigen::Vector4f ( 30, 7, 5, 1));
    cb.filter(*cloud_cropped); 
}

// segments and filters the cloud to separate the plane (road) from the rest of the cloud
void RANSAC_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_plane){
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(PLANE_DIST_TRESHOLD);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    
    // Create and get coefficients and inliers of the plane model
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    
    int num_original_cloud_points = (int) cloud->size();
    int num_iterations = 0;

    /*
        The following loop executes the segmentation algorithm until the cloud size reachs a defined treshold.
        This approach would be useful in case more than one plane was present in the scene. Since in both dataset 1 and 2
        the only present plane is the road this cycle shall execute always just once. 
    */
    while (cloud->size () > SEGMENTATION_TRESHOLD * num_original_cloud_points){
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        /*
            The following lines do:
                1. set the starting cloud from which to extract the segmented points
                2. set the indices of the points to extract
                3. set the behaviour of the filter 
                    - false -> retrieve a cloud composed of just inliers points
                    - true -> retrieve a cloud composed of the original cloud - inliers points
        */
        extract.setInputCloud(cloud); 
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_plane);
        std::cerr << "PointCloud representing the planar component: " << cloud_plane->width * cloud_plane->height << " data points." << std::endl;
        extract.setNegative(true);
        extract.filter(*cloud);
        std::cerr << "PointCloud representing the non-planar component: " << cloud->width * cloud->height << " data points." << std::endl;

        num_iterations++;
    }
    std::cerr << "Segmentation completed after " << num_iterations << " iterations." <<std::endl;
}

// retrieves cluster from a cloud
void cluster_extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<pcl::PointIndices>& cluster_indices){
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    #ifdef USE_PCL_LIBRARY
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclidean_cluster;

        // if you take a very small value, it can happen that an actual object can be seen as multiple clusters. On the other hand, if you set the value too high, it could happen, that multiple objects are seen as one cluster
        euclidean_cluster.setClusterTolerance(CLUSTER_TOLERANCE);

        // we impose that the clusters found must have at least setMinClusterSize() points and maximum setMaxClusterSize() points
        euclidean_cluster.setMinClusterSize(MIN_CLUSTER_SIZE);
        euclidean_cluster.setMaxClusterSize(MAX_CLUSTER_SIZE);
        euclidean_cluster.setSearchMethod(tree);
        euclidean_cluster.setInputCloud(cloud);
        euclidean_cluster.extract(cluster_indices);

    #else
        my_pcl::KdTree treeM;
        treeM.set_dimension(3);
        setupKdtree(cloud, &treeM, 3);
        cluster_indices = euclideanCluster(cloud, &treeM, CLUSTER_TOLERANCE, MIN_CLUSTER_SIZE, MAX_CLUSTER_SIZE);
    #endif

    std::cerr << "Clusters correctly extraxted: " << cluster_indices.size() << std::endl;

}

// computes the distance of a point wrt ego vehicle
float distance_from_ego(pcl::PointXYZ point){
    float distance = sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z,2));

    return distance;
}

/* 
    Given a box representing a cluster, this gets the nearest box vertex to that cluster.
    Please note that this function considers that the ego vehicle is in the position <0,0,0>, this is the reason why abs() is used.
    Please note that this function computes the nearest point among the vertices only.
*/
pcl::PointXYZ get_nearest_point(const Box &box){
    pcl::PointXYZ nearest;

    if(abs(box.x_min) <= abs(box.x_max))
        nearest.x = box.x_min;
    else
        nearest.x = box.x_max;

    if(abs(box.y_min) <= abs(box.y_max))
        nearest.y = box.y_min;
    else
        nearest.y = box.y_max;

    if(abs(box.z_min) <= abs(box.z_max))
        nearest.z = box.z_min;
    else
        nearest.z = box.z_max;

    return nearest;
}

// renders clusters and boxes around them
void render_clusters(Renderer& renderer, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::vector<pcl::PointIndices>& cluster_indices){
    int j = 0;
    int clusterId = 0;

    /*
        The following lines do:
            1. iteration over cluster indices. Each one represents a single cluster
            2. iterarion over the points belonging to one same cluster
            3. each of these point is pushed in a new cloud, representing the current cluster
            4. rendering of the cluster and a box around it
    */
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->push_back((*cloud)[*pit]); 
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        
        // rendering of the cloud representing the cluster
        renderer.RenderPointCloud(cloud_cluster,"cluster_"+std::to_string(clusterId),Color(0,0,1));

        pcl::PointXYZ minPt, maxPt;
        pcl::getMinMax3D(*cloud_cluster, minPt, maxPt);
        Box box{
            minPt.x, minPt.y, minPt.z,
            maxPt.x, maxPt.y, maxPt.z
        };

        // rendering of the cluster distance from ego vehicle
        pcl::PointXYZ nearest = get_nearest_point(box);
        float distance = distance_from_ego(nearest);
        renderer.addText(minPt.x, minPt.y, minPt.z,std::to_string(distance));
        renderer.addLine(pcl::PointXYZ(0,0,0), nearest, Color(1,0,0), "line_"+std::to_string(clusterId));

        /*
            A yellow box is rendered around the cluster if it is less than 5 meters away from ego vehicle (front), a red box is rendered otherwise.
            A cluster is considered to be in front of the ego vehicle if both x_min and x_max are greater than zero.
        */ 
        if(minPt.x > 0 && maxPt.x > 0 && distance <= 5)
            renderer.RenderBox(box, j, Color(1,1,0));
        else
            renderer.RenderBox(box, j, Color(1,0,0));

        ++clusterId;
        j++;
    }
}

void ProcessAndRenderPointCloud (Renderer& renderer, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1), Color(1,0,1), Color(0,1,1)};

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    downsample(cloud, cloud_filtered);

    crop(cloud_filtered);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    RANSAC_segmentation(cloud_filtered, cloud_plane);

    std::vector<pcl::PointIndices> cluster_indices;
    cluster_extraction(cloud_filtered, cluster_indices);

    render_clusters(renderer, cloud_filtered, cluster_indices);
    
    renderer.RenderPointCloud(cloud_plane,"cloud_plane", Color(0,1,0));
    // renderer.RenderPointCloud(cloud_filtered,"cloud_filtered", Color(0,0,1));
    // renderer.RenderPointCloud(cloud,"cloud", Color(0,0,1));
}

int main(int argc, char* argv[])
{
    if(argc < 2){
        std::cerr << "No arguments provided. Exiting." << std::endl;
        return 1;
    }

    Renderer renderer;
    renderer.InitCamera(CameraAngle::XY);
    // Clear viewer
    renderer.ClearViewer();

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    std::vector<boost::filesystem::path> stream(boost::filesystem::directory_iterator{argv[1]},
                                                boost::filesystem::directory_iterator{});

    // sort files in ascending (chronological) order
    std::sort(stream.begin(), stream.end());

    auto streamIterator = stream.begin();

    while (not renderer.WasViewerStopped())
    {
        renderer.ClearViewer();

        pcl::PCDReader reader;
        reader.read (streamIterator->string(), *input_cloud);
        auto startTime = std::chrono::steady_clock::now();

        ProcessAndRenderPointCloud(renderer,input_cloud);
        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "[PointCloudProcessor<PointT>::ReadPcdFile] Loaded "
        << input_cloud->points.size() << " data points from " << streamIterator->string() <<  "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        renderer.SpinViewerOnce();

        usleep(1000 * 50); //debug
    }
}
