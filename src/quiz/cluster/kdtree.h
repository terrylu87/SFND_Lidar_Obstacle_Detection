/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <utility>

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
        int dim = point.size();
        Node *n = new Node(point,id);
        if(root==NULL){
            root = n;
        }else{
            int depth = 0;
            //bool inserted = false;
            Node *current_node = root;
            while(1){
                int idx=depth%dim;
                if(point[idx] < current_node->point[idx]){
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

    // range search
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
        int dims = target.size();
        // [[lower boudary , higher boudary], [], ...]
        std::vector<std::vector<float> > threshs;
        int i;
        for(i=0;i<dims;++i)
        {
            std::vector<float> th;
            th.push_back(target[i] - distanceTol);
            th.push_back(target[i] + distanceTol);
            threshs.push_back(th);
        }

		std::vector<int> ids;
        std::vector<std::pair <Node*, int> > node_stack;
        node_stack.push_back(std::make_pair(root, 0));

        while(!node_stack.empty())
        {
            auto node = node_stack.back();

            node_stack.pop_back();

            int axis = node.second;
            int next_axis = (node.second + 1) % dims;


            bool inside_box = true;
            for(i=0;i<dims;++i)
            {
                if(node.first->point[i] < threshs[i][0]
                   || node.first->point[i] > threshs[i][1]){
                    inside_box = false;
                    break;
                }
            }

            if(inside_box){
                float dist = distance(node.first->point,target);
                if(dist <= distanceTol){
                    // record it's id
                    if(node.first->point[0]!=target[0] || node.first->point[1]!=target[1]){
                        ids.push_back(node.first->id);
                    }
                }
            }

            // explore left and right based on current axis
            if((target[axis]+distanceTol) > node.first->point[axis]){
                if(node.first->right){
                    node_stack.push_back(std::make_pair(
                                             node.first->right,
                                             next_axis));
                }
            }
            if((target[axis]-distanceTol) < node.first->point[axis]){
                if(node.first->left){
                    node_stack.push_back(std::make_pair(
                                             node.first->left,
                                             next_axis));
                }
            }
        }
        return ids;
    }
};




