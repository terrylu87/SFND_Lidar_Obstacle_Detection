/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
        Node *n = new Node(point,id);
        if(root==NULL){
            root = n;
        }else{
            int depth = 0;
            //bool inserted = false;
            Node *current_node = root;
            while(1){
                int idx=depth%2;
                if(point[idx] < current_node->point[idx]){
                    //if(point[idx] < (current_node->point)[idx]){
                    // go left
                    if(current_node->left == NULL){
                        current_node->left = n;
                        break;
                    }else{
                        current_node = current_node->left;
                        ++depth;
                        //continue;
                    }
                }else{
                    if(current_node->right == NULL){
                        current_node->right = n;
                        break;
                    }else{
                        current_node = current_node->right;
                        ++depth;
                        //continue;
                    }
                }
            }
        }

    }

    float distance(std::vector<float> point0, std::vector<float> point1)
    {
        int dims = point0.size();
        float dist = 0;
        int i;
        for(i=0;i<dims;++i){
            dist += pow(point0[i]-point1[i],2);
        }
        dist = sqrt(dist);
        return dist;
    }

    // return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
        int idx=0;
        std::vector<boost::shared_ptr<Node> > frontier[2];
        boost::shared_ptr<Node> root_ptr(root);
        frontier[idx].push_back(root_ptr);
        // BFS search
        while(frontier[idx].size() != 0){
            int sidx;
            sidx = (idx == 0 ? 1 : 0);
            for(auto ptr : frontier[idx]){
                if(distance(ptr->point,target) < distanceTol){
                    ids.push_back(ptr->id);
                    if(ptr->left != NULL){
                        frontier[sidx].push_back(boost::shared_ptr<Node> (ptr->left));
                    }
                    if(ptr->right != NULL){
                        frontier[sidx].push_back(boost::shared_ptr<Node> (ptr->right));
                    }
                }
            }
            frontier[idx].clear();
            idx = sidx;
        }
		return ids;
	}

};




