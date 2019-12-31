// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"

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
            if (*node == NULL)
                *node = new Node(point, id);
            else
            {
                // calculate current dimension
                uint cd = depth%3;
                if (point[cd] < ((*node)->point[cd]))
                    insertHelper(&((*node)->left), depth+1, point, id);
                else
                    insertHelper(&((*node)->right), depth+1, point, id);
            }
        }

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
            insertHelper(&root, 0, point, id);
	}

        void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
        {
            if (node != NULL)
            {
                if ((node->point[0] >= (target[0]-distanceTol) &&
                    node->point[0] <= (target[0]+distanceTol)) &&
                    (node->point[1] >= (target[1]-distanceTol) &&
                    node->point[1] <= (target[1]+distanceTol)) &&
                    (node->point[2] >= (target[2]-distanceTol) &&
                    node->point[2] <= (target[2]+distanceTol)))
                {
                    float distance = sqrt((node->point[0]-target[0])*(node->point[0]-target[0]) +
                                          (node->point[1]-target[1])*(node->point[1]-target[1]) +
                                          (node->point[1]-target[1])*(node->point[1]-target[1]));
                    if (distance <= distanceTol)
                        ids.push_back(node->id);
                }
                // check across boundary
                if ((target[depth%3]-distanceTol) < node->point[depth%3])
                    searchHelper(target, node->left, depth+1, distanceTol, ids);
                if ((target[depth%3]+distanceTol) > node->point[depth%3])
                    searchHelper(target, node->right, depth+1, distanceTol, ids);
            }            
        }


	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
	    std::vector<int> ids;
            searchHelper(target, root, 0, distanceTol, ids);

	    return ids;
	}
	

};
template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
    //void clusterHelper(int indice, const std::vector<std::vector<float>>& points, 
    void clusterHelper(int indice, typename pcl::PointCloud<PointT>::Ptr cloud, 
                       std::vector<int>& cluster, 
                       std::vector<bool>& processed,
                       KdTree* tree, float distanceTol)
    {
            processed[indice] = true;
            cluster.push_back(indice);
            std::vector<float> point;
            point.push_back((*cloud)[indice].x);
            point.push_back((*cloud)[indice].y);
            point.push_back((*cloud)[indice].z);
            std::vector<int> nearest = tree->search(point, distanceTol);
            for (int id: nearest)
            {
                    if (!processed[id])
                            clusterHelper(id, cloud, cluster, processed, tree, distanceTol);
            }
    }

    //std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
    std::vector<std::vector<int>> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol, 
                                                float minSize)
    {

            // TODO: Fill out this function to return list of indices for each cluster

            std::vector<std::vector<int>> clusters;

            //std::vector<bool> processed(points.size(), false);
            std::vector<bool> processed(cloud->size(), false);
            
            int i = 0;
            //while (i < points.size())
            while (i < cloud->size())
            {
                    std::cout << "Checking point " << i;
                    if (processed[i])
                    {
                            i++;
                            continue;
                    }
                    std::vector<int> cluster;
                    //clusterHelper(i, points, cluster, processed, tree, distanceTol);
                    clusterHelper(i, cloud, cluster, processed, tree, distanceTol);
                    if (cluster.size() >= minSize)
                            clusters.push_back(cluster);
                    i++;
            }
            return clusters;
    }
};
#endif /* PROCESSPOINTCLOUDS_H_ */
