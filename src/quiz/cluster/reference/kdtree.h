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

	void insertHelper(Node** node, uint depth, std::vector<float> point, int id)
	{

		// Tree is empty
		if(*node==NULL)
			*node = new Node(point,id);
		else
		{
			// Calculate current dim
			uint cd = depth % 2;
	
			if(point[cd] < ((*node)->point[cd]))
				insertHelper(&((*node)->left), depth+1, point, id);
			else
				insertHelper(&((*node)->right), depth+1, point, id);
		}

	}

	void insert(std::vector<float> point, int id)
	{
		insertHelper(&root,0,point,id);
	}

	void searchHelper(std::vector<float> pivot, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{

		if(node!=NULL)
		{
			if( (node->point[0]>=(pivot[0]-distanceTol)&&node->point[0]<=(pivot[0]+distanceTol)) && (node->point[1]>=(pivot[1]-distanceTol)&&node->point[1]<=(pivot[1]+distanceTol)) )
			{
				float distance = sqrt((node->point[0]-pivot[0])*(node->point[0]-pivot[0])+(node->point[1]-pivot[1])*(node->point[1]-pivot[1]));
				if(distance <= distanceTol)
					ids.push_back(node->id);
			}

			//check accross boundary
			if((pivot[depth%2]-distanceTol)<node->point[depth%2])
				searchHelper(pivot,node->left,depth+1,distanceTol,ids);
			if((pivot[depth%2]+distanceTol)>node->point[depth%2])
				searchHelper(pivot,node->right,depth+1,distanceTol,ids);
			
		}

	}

	// return a list of point ids in the tree that are within distance of pivot
	std::vector<int> search(std::vector<float> pivot, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(pivot, root, 0, distanceTol, ids);

		return ids;
	}
	

};




