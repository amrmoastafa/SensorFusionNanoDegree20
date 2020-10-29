/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

// Color macros & Test Macro
#define BLUE Color(0,0,1)
#define WHITE Color(1,1,1)
#define RED Color(1,0,0)
#define GREEN Color(0,1,0)
#define MIXED_1 Color(1,1,0)
#define MIXED_2 Color(0,1,1)
#define MIXED_3 Color(1,0,1)
#define TEST 0

// Segmentation macros
#define SEG_MAX_ITER 100
#define SEG_THRESHOLD 0.35

// Clustering macros
#define CLUSTER_MIN_SIZE 10
#define CLUSTER_MAX_SIZE 3000
#define CLUSTER_TOLERANCE 0.4

// Filtering macros
#define VOXEL_GRID_SIZE 0.15


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

    // renderPointCloud(viewer,inputCloud,"inputCloud");

   // Filter point cloud with PCL built-in functions
    auto filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.3, Eigen::Vector4f(-20, -10.0, -3, 1), Eigen::Vector4f(25, 10.0, 8, 1));
    // renderPointCloud(viewer, filterCloud, "filterCloud");

    // Segment the road plane
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, SEG_MAX_ITER, SEG_THRESHOLD);
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));
    // renderPointCloud(viewer, inputCloud, "inputCloud", Color(1, 1, 1));

    // Cluster obstacles
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.4, 30, 3000);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1), Color(1,0,1), Color(0,1,1), Color(1,1,1)};
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) {
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstacleCloud"+std::to_string(clusterId), colors[clusterId%colors.size()]);

        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }

  
}


std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar *lidar = new Lidar{cars,0.0};
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_OutputRays = lidar->scan();//input cloud
    //renderRays(viewer,lidar->position,lidar->scan());
    //renderRays(viewer,lidar->position,lidar_OutputRays);

    //renderPointCloud(viewer,lidar_OutputRays,"lidar_Output"); //remove this to stop rendering the ground

    // TODO:: Create point processor
    // Assigned to heap Dynamically
    ProcessPointClouds<pcl::PointXYZ> *Point_Processor = new ProcessPointClouds<pcl::PointXYZ>;
    // Assigned to stack Statically
    // ProcessPointClouds<pcl::PointXYZ> Point_Processor;

    /*                              Segmentation Part                                           */

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = Point_Processor->SegmentPlane(lidar_OutputRays,5,0.2);
    renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    // renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

    /*                                    CLustering Part                                        */
    
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = Point_Processor->Clustering(segmentCloud.first, 3.0, 3, 30);
    
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(1,1,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        Point_Processor->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);

        Box box = Point_Processor->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        // We need to search for a way to draw bounding boxed that have z-values and are a little rotated
        ++clusterId;
    }

  
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}

void test(pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    ProcessPointClouds<pcl::PointXYZI>* pointCloudProcessor = new ProcessPointClouds<pcl::PointXYZI>();
    typename pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointCloudProcessor->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");


    Eigen::Vector4f minPoint (-30, -6.5, -3, 1);
    Eigen::Vector4f maxPoint (30, 6.5, 8, 1);

    // Filtering the cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterdCloud = pointCloudProcessor->FilterCloud(inputCloud, VOXEL_GRID_SIZE, minPoint, maxPoint);

    // Segmenting the cloud to obstacles & plane
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointCloudProcessor->SegmentPlane(filterdCloud, SEG_MAX_ITER, SEG_THRESHOLD);

    renderPointCloud(viewer, segmentCloud.first, "Obstacles Rendering", BLUE);
    renderPointCloud(viewer, segmentCloud.second, "Plane Rendering", RED);

/*     // Clustering obstacles object
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>  cloudClusters = pointCloudProcessor->Clustering(segmentCloud.first,CLUSTER_TOLERANCE, CLUSTER_MIN_SIZE, CLUSTER_MAX_SIZE);


    std::vector<Color> colors = {MIXED_1, MIXED_2, MIXED_3};
    int clusterId = 0;
    float boxOpacity = 1;
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster: cloudClusters){

        // std::cout << "Cluster size: " << processPntCld.numPoints(cluster) << std::endl;
        // renderPointCloud(viewer, cluster, "Cluster " + std::to_string(clusterId), colors[clusterId%3]);
        // Rendering a box 
        Box box = pointCloudProcessor->BoundingBox(cluster);
        // renderBox(viewer, box, clusterId, colors[clusterId%3], boxOpacity);
        clusterId++;
    }
 */
}

int main (int argc, char** argv)
{

    std::cout << "starting enviroment" << std::endl;
    
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    // cityBlock(viewer, pointProcessorI, inputCloudI);
    if(TEST)
    {   
        // Testing via single point cloud frame
        test(viewer);
        while( !viewer->wasStopped() )
            viewer->spinOnce();    
    }

    else
    {
        while (!viewer->wasStopped ())
        {

        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
        }
    }

}