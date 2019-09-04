/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

#include <iostream>
using namespace std;
std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
    int i;
    int cloud_size = cloud->size();
    int current_max_fit = 0;
    for(i=0;i<maxIterations;++i){
        // Randomly sample subset and fit line
        int rand0 = rand()%cloud_size;
        int rand1 = rand()%cloud_size;
        pcl::PointXYZ p0 = cloud->at(rand0);
        pcl::PointXYZ p1 = cloud->at(rand1);
        //cout << "size = " << cloud_size << endl;
        //cout << "r0 = " << rand0 << ", r1 = " << rand1 << endl;
        //cout << p0.x;
        float A = p0.y - p1.y;
        float B = p1.x - p0.x;
        float C = p0.x * p1.y - p1.x * p0.y;

        int count=0;
        std::unordered_set<int> inliers;
        // Measure distance between every point and fitted line
        int j;
        for(j=0;j<cloud_size;++j){
            pcl::PointXYZ point = cloud->at(j);
            float div = sqrt(A*A+B*B);
            if(div > 0){
                float distance = abs(A*point.x+B*point.y+C) / div;
                // If distance is smaller than threshold count it as inlier
                if(distance < distanceTol){
                    ++count;
                    inliers.insert(j);
                }
                //cout << "distance = " << distance << endl;
            }
        }
        if(count > current_max_fit){
            current_max_fit = count;
            inliersResult = inliers;
        }
    }

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}


std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
    int i;
    int cloud_size = cloud->size();
    int current_max_fit = 0;
    for(i=0;i<maxIterations;++i){
        // Randomly sample subset and fit line
        pcl::PointXYZ p0 = cloud->at(rand()%cloud_size);
        pcl::PointXYZ p1 = cloud->at(rand()%cloud_size);
        pcl::PointXYZ p2 = cloud->at(rand()%cloud_size);
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
        std::unordered_set<int> inliers;
        // Measure distance between every point and fitted line
        int j;
        for(j=0;j<cloud_size;++j){
            pcl::PointXYZ point = cloud->at(j);
            float div = sqrt(A*A+B*B+C*C);
            if(div > 0){
                float distance = abs(A*point.x+B*point.y+C*point.z+D) / div;
                // If distance is smaller than threshold count it as inlier
                if(distance < distanceTol){
                    ++count;
                    inliers.insert(j);
                }
                //cout << "distance = " << distance << endl;
            }
        }
        if(count > current_max_fit){
            current_max_fit = count;
            inliersResult = inliers;
        }
    }

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;

}


int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	//std::unordered_set<int> inliers = Ransac(cloud, 5, 0.5);
	std::unordered_set<int> inliers = Ransac3D(cloud, 10, 0.1);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
