#ifndef SEARCH_H
#define SEARCH_H

#include <vector>
#include "./quiz/cluster/kdtree.h"


void proximity(const int point_id, std::vector<int>& cluster,
               std::set<int>& checked_list,
               const std::vector<std::vector<float>>& points,
               KdTree* tree,float distanceTol);

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol);

#endif