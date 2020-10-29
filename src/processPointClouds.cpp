// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


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

    // Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);

    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);

    // Use CropBox to set the extent of our region of interest
    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);

    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    // Use CropBox to find the roof interference points
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);

    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1  ,  1));
    roof.setMax(Eigen::Vector4f ( 2.6,  1.7, -0.4,  1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    // Add these roof points to a pcl::PointIndices object
    pcl::PointIndices::Ptr roofPoints {new pcl::PointIndices};
    for (int point : indices) {
        roofPoints->indices.push_back(point);
    }

    // Remove these roof points from cloudRegion (the region of interest)
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(roofPoints);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(std::unordered_set<int>& inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr Obstacles_PCL(new pcl::PointCloud<PointT>());//obstCloud
    typename pcl::PointCloud<PointT>::Ptr SegPlane_PCL(new pcl::PointCloud<PointT>());//planeCloud


	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliers.count(index))
			SegPlane_PCL->points.push_back(point);
		else
			Obstacles_PCL->points.push_back(point);
	}

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(Obstacles_PCL, SegPlane_PCL);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	std::unordered_set<int> inliersResult;

	
	while(maxIterations--)
	{
		
		std::unordered_set<int> inliers;
		while(inliers.size()<3)//picking three random points to form our plane instead of 2 for straight line
		{
			inliers.insert(rand()%(cloud->points.size()));
		}
		float x1, y1, z1, x2, y2, z2, x3, y3, z3;
		float a, b, c, d, Vect3_i, Vect3_j, Vect3_k;
		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;
		// Given three points , we will need to calculate the normal vector using equations for i , j , k
		// We need to get vector A from point 1 to point 2 , vector B from point 1 to point 3 , then cross product them in vect 3
		Vect3_i = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
		Vect3_j = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
		Vect3_k = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);

		// Equation of plane : Ax + By + Cz + D = 0
		a = Vect3_i;
		b = Vect3_j;
		c = Vect3_k;
		d = -(Vect3_i*x1 + Vect3_j*y1 + Vect3_k*z1);

		for (int index = 0; index < cloud->points.size(); index++) {
			// Skip if the considered point is already an inlier.
			if (inliers.count(index) > 0) continue;
			
			PointT point = cloud->points[index];
			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;
			float distance = fabs(a*x4 + b*y4 + c*z4 + d) / sqrt(a*a + b*b + c*c);

			if (distance <= distanceThreshold) {
				inliers.insert(index);
			}
		}

		// Return indicies of inliers from fitted line with most inliers
		if (inliers.size() > inliersResult.size()) {
			inliersResult = inliers;
		}
	}


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliersResult,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    
    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance); // 2cm
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for(pcl::PointIndices getIndices: cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
        for(int index : getIndices.indices)
        {
            cloudCluster->points.push_back(cloud->points[index]);
        }
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;
        clusters.push_back(cloudCluster);
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