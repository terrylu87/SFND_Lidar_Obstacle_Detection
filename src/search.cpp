#include "search.h"

void proximity(const int point_id, std::vector<int>& cluster,
               std::set<int>& checked_list,
               const std::vector<std::vector<float>>& points,
               KdTree* tree,float distanceTol)
{
    checked_list.insert(point_id);
    cluster.push_back(point_id);
    auto nearby_points(tree->search(points[point_id],distanceTol));
    int i;
    for(i=0;i<nearby_points.size();++i)
    {
        if(checked_list.find(nearby_points[i]) == checked_list.end()){
            proximity(nearby_points[i],cluster,checked_list,points,tree,distanceTol);
        }
    }
}


std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster
	std::vector<std::vector<int>> clusters;

    std::set<int> checked_list;
    //clusters
    int i;
    for(i=0;i<points.size();++i)
    {
        if(checked_list.find(i) == checked_list.end()){
            std::vector<int> cluster;
            proximity(i,cluster,checked_list,points,tree,distanceTol);
            clusters.push_back(cluster);
        }
    }
	return clusters;

}