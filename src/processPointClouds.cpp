// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

/////
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
        
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    typename pcl::PointCloud<PointT>::Ptr cloud_voxel (new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr cloud_io (new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr cloud_io_roof (new pcl::PointCloud<PointT>);

    ///Downsample through Voxel Grid
    typename pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes,filterRes,filterRes);
    vg.filter(*cloud_voxel);


    typename pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
    /// Filter --> Passthrough filter (Works just delete all points above plane)
/*    typename pcl::PassThrough<PointT> pt;
    pt.setInputCloud(cloud_filtred_voxel);
    pt.setFilterFieldName("x");
    pt.setFilterLimits(0.2,1);
    //pt.setFilterLimitsNegative(true);
    pt.filter(*filtered);*/
    /// Filter --> Conditional removal (need to find a good condition to perform)
/*    typename pcl::ConditionAnd<PointT>::Ptr condition(new pcl::ConditionAnd<PointT>);
    condition->addComparison(typename pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>("x", pcl::ComparisonOps::GT, minPoint[0])));
    condition->addComparison(typename pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>("x", pcl::ComparisonOps::LT, maxPoint[0])));
    //condition->addComparison(typename pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>("y", pcl::ComparisonOps::GT, minPoint[1])));
    //condition->addComparison(typename pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>("y", pcl::ComparisonOps::LT, maxPoint[1])));

    // Filter Object
    typename pcl::ConditionalRemoval<PointT> cr(true);
    cr.setInputCloud(cloud_filtred_voxel);
    cr.setCondition(condition);
    //cr.setKeepOrganized(false);
    //cr.setUserFilterValue(0.0);
    cr.filter(*filtered);*/

    /// Outlier removing --> Radius based ( Kind of works)
/*    typename pcl::RadiusOutlierRemoval<PointT> filter;
    filter.setInputCloud(cloud_filtred_voxel);
    // Every point must have 10 neighbors within 15cm, or it will be removed.
    filter.setRadiusSearch(0.15);
    filter.setMinNeighborsInRadius(15);
    filter.filter(*filtered);*/

    /// Extract everything outside of the box (box dimension given according to minPoint and maxPoint)
    pcl::CropBox<PointT> cb(true);   // true -> because we are dealing with points inside the box.
    cb.setMin(minPoint);
    cb.setMax(maxPoint);
    cb.setInputCloud(cloud_voxel);
    cb.filter(*cloud_io);

    /// Outlier removing --> Statistical based on gaussian (normal) distribution  "" Worked Perfectly"" ***
/*    typename pcl::StatisticalOutlierRemoval<PointT> filter;
    filter.setInputCloud(cloud_io);
    // Set number of neighbors to consider to 50.
    filter.setMeanK(50);
    // Set standard deviation multiplier to 1.
    // Points with a distance larger than 1 standard deviation of the mean distance will be outliers.
    filter.setStddevMulThresh(1.0);
    filter.filter(*filtered);*/

    /// (Optional) Alternatively we can remove the lidar heading points from roof by using also CroBox
    std::vector<int> indices;  // indices of roof

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7,-1,1));
    roof.setMax(Eigen::Vector4f(2.6,1.7,-.4,1));
    roof.setInputCloud(cloud_io);
    roof.filter(indices);

    /// Extract roof cloud points indices from main cloud
    // Create the filtering objects
    pcl::ExtractIndices<PointT> extract;

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
     for (int index : indices)
         inliers->indices.push_back(index);

     typename pcl::ExtractIndices<PointT> extract_roof;
     extract_roof.setIndices(inliers);
     extract_roof.setInputCloud(cloud_io);
     extract_roof.setNegative(true);
     extract_roof.filter(*filtered);





    /// From Robotica calculation of Normals
    /*  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

     pcl::NormalEstimation<PointT,pcl::Normal> normalEstimation;
     normalEstimation.setInputCloud(cloud);
     // For every point, use all neighbors in a radius of 3cm.
     normalEstimation.setRadiusSearch(0.5);
     // A kd-tree is a data structure that makes searches efficient. More about it later.
     // The normal estimation object will use it to find nearest neighbors.
     typename pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
     normalEstimation.setSearchMethod(kdtree);
     //Calculate the normals;
     normalEstimation.compute(*normals);
     // Visualize them.
     boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
     viewer->addPointCloud<PointT>(cloud, "cloud");
     // Display one normal out of 20, as a line of length 3cm.
     viewer->addPointCloudNormals<PointT, pcl::Normal>(cloud, normals, 20, 0.03, "normals");*/

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return filtered;
    //return cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr cloud_obstacles (new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr cloud_roads (new pcl::PointCloud<PointT>);


    // Create the filtering objects 


    //Udacity teacher version
     for (int index : inliers->indices)
          cloud_roads->points.push_back(cloud->points[index]);
          pcl::ExtractIndices<PointT> extract;
          extract.setInputCloud(cloud);
          extract.setIndices(inliers);
          extract.setNegative(true);
          extract.filter(*cloud_obstacles);


      /// Alternative calculation way
/*
    pcl::ExtractIndices<PointT> extract;
    if (inliers->indices.size ()== 0 )
    {
        std::cerr<<"could not estimate any indices"<<std::endl;
    }
    else 
	{
        // Extrcat the inliers 
        extract.setInputCloud(cloud);
        extract.setIndices(inliers); // in order to extract the indices which are basically generated by SACMODE_PLANE and belong to plane 
        // resulting cloud_roads contains all points of cloud that are indexed by inliers
        extract.filter(*cloud_roads);
        extract.setNegative(true);
        // resulting cloud_obstacles contatins ONLY Obstacles points of cloud.
        extract.filter(*cloud_obstacles);
        *//*if we set extract.setNegative(FALSE),
          output from extract.filter will be cloud_roads (because of MODEL_PLANE)
    }*/


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_obstacles, cloud_roads);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.

    // Initialize model coefficients and inliers
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());  // define what the plane is, and can be used if you want to render this plane in the pcl viewer
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());   // Output of RANSAC basically contains 2 datas, 1-) Inlier, the data adjust to the defined model. 2-) Outlier, does not fit to model.
                                                                // later we will use this Inliers to seperate the point cloud into 2 peaces.
    //Create the segmentation object
    typename pcl::SACSegmentation<PointT> seg;
    //Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory hyperparameters for seg object
    seg.setMethodType(pcl::SACMODEL_PLANE);
    seg.setModelType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    /* Segment the largest planar component from the remaining cloud  */
    seg.setInputCloud(cloud);
    seg.segment(*inliers,*coefficients); // Here inliers clouds will be road.
    if (inliers->indices.size ()== 0 )
    {
        std::cerr<<"could not estimate a planar model for the given dataset "<<std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    typename pcl::search::KdTree<PointT>::Ptr tree (new typename pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    typename pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    std::vector<pcl::PointIndices> clusterIndices;
    ec.extract(clusterIndices);

    //for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); it++ )
     for (pcl::PointIndices getIndices: clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster ( new pcl::PointCloud<PointT>);
        for (int index : getIndices.indices)
            cloudCluster->points.push_back(cloud->points[index]);

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;
        clusters.push_back(cloudCluster);

        std::cout << "PointCloud representing the Cluster: " << cloudCluster->size () << " data points." << std::endl;
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());


    return paths;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    auto startTime= std::chrono::steady_clock::now();
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    // Randomly sample subset and fit line


    // Measure distance between every point and fitted line
    // If distance is smaller than threshold count it as inlier

    // Return indicies of inliers from fitted line with most inliers


    // TODO: Fill in this function
    while(maxIterations--)
    {
        // randomly pick two points
        std::unordered_set<int> inliers;
        while (inliers.size()<3)
            inliers.insert(rand()%(cloud->points.size()));
        float x1,y1,x2,y2,z1,z2,x3,y3, z3;

        auto itr = inliers.begin();
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
        itr ++;
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
        itr ++;
        x3 = cloud->points[*itr].x;
        y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;

        std::vector<float> v1 = {(x2-x1),(y2-y1),(z2-z1)};
        std::vector<float> v2 = {(x3-x1),(y3-y1),(z3-z1)};

        std::vector<float> cros_prod = {(y2-y1)*(z3-z1)-(z2-z1)*(y3-y1),
                                        (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1),
                                        (x2-x1)*(y3-y1)-(y2-y1)*(x2-x1)};

        float ii = cros_prod[0];
        float jj = cros_prod[1];
        float kk = cros_prod[2];
        std::complex<float> i,j,k;
        std::complex<float> d = -(x1*ii + jj*y1 + kk*z1);

        for (int index = 0 ; index <cloud->points.size() ; index ++)
        {
            if (inliers.count(index)>0)
                continue;
             pcl::PointXYZI point = cloud->points[index];
            float x4 = point.x;
            float y4 = point.y;
            float z4 = point.z;

            float dist = fabs(ii*x4+jj*y4+kk*z4+d)/sqrt(ii*ii +jj*jj+kk*kk);
            if (dist <= distanceTol)
                inliers.insert(index);
        }
        if (inliers.size()>inliersResult.size())
            inliersResult= inliers;

    }


    auto endTime= std::chrono::steady_clock::now();
    auto elapseTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
    std::cout<< "RANSAC Algorthm took: "<< elapseTime.count()<<"microseconds"<<std::endl;

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    for (auto i : inliersResult) {
        inliers->indices.push_back(i);
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

    return segResult;
}


