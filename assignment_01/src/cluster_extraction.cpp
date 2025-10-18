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
#include "../include/Renderer.hpp"
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <chrono>
#include <unordered_set>
#include "../include/tree_utilities.hpp"
#include <boost/filesystem.hpp>
#include <unistd.h>

namespace fs = boost::filesystem;

using namespace lidar_obstacle_detection;

// #define USE_PCL_LIBRARY
#define DATASET_PATH "../dataset_1"

typedef std::unordered_set<int> my_visited_set_t;

//This function sets up the custom kdtree using the point cloud
void setupKdtree(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, my_pcl::KdTree* tree, int dimension)
{
    //insert point cloud points into tree
    for (int i = 0; i < cloud->size(); ++i)
    {
        tree->insert({cloud->at(i).x, cloud->at(i).y, cloud->at(i).z}, i);
    }
}

/*
OPTIONAL
This function computes the nearest neighbors and builds the clusters
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
void proximity(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int target_ndx, my_pcl::KdTree* tree, float distanceTol, my_visited_set_t& visited, std::vector<int>& cluster, int max)
{
	if (cluster.size() < max)
    {
        cluster.push_back(target_ndx);
        visited.insert(target_ndx);

        std::vector<float> point {cloud->at(target_ndx).x, cloud->at(target_ndx).y, cloud->at(target_ndx).z};
    
        // get all neighboring indices of point
        std::vector<int> neighborNdxs = tree->search(point, distanceTol);

        for (int neighborNdx : neighborNdxs)
        {
            // if point was not visited
            if (visited.find(neighborNdx) == visited.end())
            {
                proximity(cloud, neighborNdx, tree, distanceTol, visited, cluster, max);
            }

            if (cluster.size() >= max)
            {
                return;
            }
        }
    }
}

/*
OPTIONAL
This function builds the clusters following a euclidean clustering approach
    - Input:
        + cloud: Point cloud to be explored
        + tree: kd tree for searching neighbors
        + distanceTol: Distance tolerance to build the clusters 
        + setMinClusterSize: Minimum cluster size
        + setMaxClusterSize: Max cluster size
    - Output:
        + cluster: at the end of this function we will have a set of clusters
TODO: Complete the function
*/
std::vector<pcl::PointIndices> euclideanCluster(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, my_pcl::KdTree* tree, float distanceTol, int setMinClusterSize, int setMaxClusterSize)
{
	my_visited_set_t visited{};                                                          //already visited points
	std::vector<pcl::PointIndices> clusters;                                             //vector of PointIndices that will contain all the clusters
    std::vector<int> cluster;                                                            //vector of int that is used to store the points that the function proximity will give me back
	//for every point of the cloud
    //  if the point has not been visited (use the function called "find")
    //    find clusters using the proximity function
    //
    //    if we have more points than the minimum
    //      Create the cluster and insert it in the vector of clusters. You can extract the indices from the cluster returned by the proximity funciton (use pcl::PointIndices)   
    //    end if
    //  end if
    //end for
    for(int i = 0; i < cloud->size(); ++i){
        if(visited.find(i) == visited.end()){
            proximity(cloud, i, tree, distanceTol, visited, cluster, setMaxClusterSize);
            if(cluster.size() >= setMinClusterSize){
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
    downsampler.setLeafSize(0.1f, 0.1f, 0.1f);
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
    seg.setDistanceThreshold(0.1);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    
    // Create and get coefficients and inliers of the plane model
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    
    int num_original_cloud_points = (int) cloud->size();
    int num_iterations = 0;

    /*
        The following loop execute the segmentation algorithm until the cloud size reachs a defined treshold.
        This approach would be useful in case more than one plane was present in the scene. Since in both dataset 1 and 2
        the only present plane is the road this cycle shall execute always just once. 
    */
    while (cloud->size () > 0.6 * num_original_cloud_points){
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        /*
            The following lines do:
                1. setting the starting cloud from which to extract the segmented points
                2. setting the indices of the points to extract
                3. setting the behaviour of the filter 
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

        //If you take a very small value, it can happen that an actual object can be seen as multiple clusters. On the other hand, if you set the value too high, it could happen, that multiple objects are seen as one cluster
        euclidean_cluster.setClusterTolerance(0.2);

        //We impose that the clusters found must have at least setMinClusterSize() points and maximum setMaxClusterSize() points
        euclidean_cluster.setMinClusterSize(100);
        euclidean_cluster.setMaxClusterSize(25000);
        euclidean_cluster.setSearchMethod(tree);
        euclidean_cluster.setInputCloud(cloud);
        euclidean_cluster.extract(cluster_indices);

    #else
        // Optional assignment
        my_pcl::KdTree treeM;
        treeM.set_dimension(3);
        setupKdtree(cloud, &treeM, 3);
        cluster_indices = euclideanCluster(cloud, &treeM, 0.2, 100, 25000);
    #endif

    std::cerr << "Clusters correctly extraxted: " << cluster_indices.size() << std::endl;

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

        // rendering of the box around the cluster
        pcl::PointXYZ minPt, maxPt;
        pcl::getMinMax3D(*cloud_cluster, minPt, maxPt);
        Box box{minPt.x, minPt.y, minPt.z,
        maxPt.x, maxPt.y, maxPt.z};
        renderer.RenderBox(box, j);

        //TODO: 8) Here you can plot the distance of each cluster w.r.t ego vehicle
        
        //TODO: 9) Here you can color the vehicles that are both in front and 5 meters away from the ego vehicle
        //please take a look at the function RenderBox to see how to color the box
        
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
    Renderer renderer;
    renderer.InitCamera(CameraAngle::XY);
    // Clear viewer
    renderer.ClearViewer();

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    std::vector<boost::filesystem::path> stream(boost::filesystem::directory_iterator{DATASET_PATH},
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
