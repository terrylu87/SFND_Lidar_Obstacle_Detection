//#ifndef PROCESSPOINTCLOUDS_CPP
//#define PROCESSPOINTCLOUDS_CPP
// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <iostream>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>
#include <pcl/filters/crop_box.h>
#include "./quiz/cluster/kdtree.h"
#include "search.h"

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
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT> );
    typename pcl::PointCloud<PointT>::Ptr cloud_croped (new pcl::PointCloud<PointT> );

    // Create the filtering object voxel grid
    pcl::VoxelGrid<PointT> sor;
    //pcl::VoxelGrid<pcl::PCLPointCloud2> sor;

    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered);

    pcl::CropBox<PointT> box;
    box.setInputCloud(cloud_filtered);
    box.setMin(minPoint);
    box.setMax(maxPoint);
    box.filter(*cloud_croped);

    // remove roof
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5,-1.7,-1,1));
    roof.setMax(Eigen::Vector4f (2.6,1.7,-0.4,1));
    roof.setInputCloud(cloud_croped);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point:indices){
        inliers->indices.push_back(point);
    }
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_croped);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_croped);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_croped;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr objCloud(new pcl::PointCloud<PointT>);
    //std::vector<pcl::PointIndices> cluster_indices;
    pcl::ExtractIndices<PointT> filter(true);
    filter.setInputCloud(cloud);
    filter.setIndices(inliers);
    filter.setNegative (false);
    filter.filter(*planeCloud);
    filter.setNegative (true);
    filter.filter(*objCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(objCloud, planeCloud);
    return segResult;
}


#include <unordered_set>
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // DONE:: Fill in this function to find inliers for the cloud.
    //pcl::PointIndices inliersResult;
    pcl::PointIndices::Ptr inliersResult(new pcl::PointIndices);
	//std::unordered_set<int> inliersResult;
	srand(time(NULL));
	// For max iterations
    int i;
    int cloud_size = cloud->size();
    int current_max_fit = 0;
    for(i=0;i<maxIterations;++i){
        // Randomly sample subset and fit line
        PointT p0 = cloud->at(rand()%cloud_size);
        PointT p1 = cloud->at(rand()%cloud_size);
        PointT p2 = cloud->at(rand()%cloud_size);
        //cout << "size = " << cloud_size << endl;
        //cout << "r0 = " << rand0 << ", r1 = " << rand1 << endl;
        //cout << p0.x;
        float x1(p0.x),x2(p1.x),x3(p2.x);
        float y1(p0.y),y2(p1.y),y3(p2.y);
        float z1(p0.z),z2(p1.z),z3(p2.z);
        float A = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
        float B = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
        float C = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
        float D = -(A*x1+B*y1+C*z1);

        int count=0;
        //std::unordered_set<int> inliers;
        pcl::PointIndices inliers;
        // Measure distance between every point and fitted line
        int j;
        for(j=0;j<cloud_size;++j){
            PointT point = cloud->at(j);
            float div = sqrt(A*A+B*B+C*C);
            if(div > 0){
                float distance = abs(A*point.x+B*point.y+C*point.z+D) / div;
                // If distance is smaller than threshold count it as inlier
                if(distance < distanceThreshold){
                    ++count;
                    inliers.indices.push_back(j);
                }
                //cout << "distance = " << distance << endl;
            }
        }
        if(count > current_max_fit){
            current_max_fit = count;
            *inliersResult = inliers;
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    auto segResult = SeparateClouds(inliersResult,cloud);
    return segResult;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // DONE:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    std::vector<std::vector<float> > points;

	KdTree* tree = new KdTree;
    for (int i=0; i<cloud->size(); ++i)
    {
        points.push_back({(*cloud)[i].x, (*cloud)[i].y, (*cloud)[i].z});
    	tree->insert({(*cloud)[i].x, (*cloud)[i].y, (*cloud)[i].z},i);
    }

    //typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    //tree->setInputCloud (cloud);

  	std::vector<std::vector<int> > cluster_indices = euclideanCluster(points,
                                                                      tree,
                                                                      clusterTolerance);

    int j = 0;
    //for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    for (std::vector<std::vector<int> >::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (std::vector<int>::const_iterator pit = it->begin (); pit != it->end (); ++pit)
        cloud_cluster->points.push_back (cloud->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        j++;
        clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::PCLClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // DONE:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); // 2cm
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->points.push_back (cloud->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        j++;
        clusters.push_back(cloud_cluster);
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
BoxQ ProcessPointClouds<PointT>::xyBoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // Compute principal directions
    typename pcl::PointCloud<PointT>::Ptr cloud_xy(new pcl::PointCloud<PointT> ());
    pcl::copyPointCloud(*cluster,*cloud_xy);
    int i;
    for(i=0;i<cloud_xy->size();++i)
    {
        cloud_xy->at(i).z = 0;
    }
    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cloud_xy, pcaCentroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*cloud_xy, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected (new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);
    // Get the minimum and maximum points of the transformed cloud.
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal =
        0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    BoxQ box;
    box.cube_length = maxPoint.x - minPoint.x;
    box.cube_width = maxPoint.y - minPoint.y;
    box.cube_height = maxPoint.z - minPoint.z;
    box.bboxTransform =
         eigenVectorsPCA* meanDiagonal + pcaCentroid.head<3>();
    box.bboxQuaternion = (eigenVectorsPCA);

    return box;
}


template<typename PointT>
BoxQ ProcessPointClouds<PointT>::xyzBoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cluster, pcaCentroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    // This line is necessary for proper orientation in some cases.
    // The numbers come out the same without it, but
    // the signs are different and the box doesn't get correctly oriented in some cases
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
    // std::cerr << std::endl << "eigenVectorsPCA: " << eigenVectorsPCA << std::endl;
    // std::cerr << std::endl << "pcaCentroid: " << pcaCentroid << std::endl;

    //typename pcl::PointCloud<PointT>::Ptr cloudPCAprojection (new pcl::PointCloud<PointT>);
    //pcl::PCA<PointT> pca;
    //pca.setInputCloud(cluster);
    //pca.project(*cluster, *cloudPCAprojection);
    // std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;
    // std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;

    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected (new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);
    // Get the minimum and maximum points of the transformed cloud.
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal =
        0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // Final transform
    // Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
    BoxQ box;
    box.cube_length = maxPoint.x - minPoint.x;
    box.cube_width = maxPoint.y - minPoint.y;
    box.cube_height = maxPoint.z - minPoint.z;
    box.bboxTransform =
         eigenVectorsPCA* meanDiagonal + pcaCentroid.head<3>();
    box.bboxQuaternion = (eigenVectorsPCA);

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

//#endif