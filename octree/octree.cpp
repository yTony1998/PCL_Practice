#include <iostream>
#include <vector>
#include <ctime>

#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>
#include <pcl/visualization/cloud_viewer.h>

int main()
{
    srand((unsigned int) time(NULL));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width=1000;
    cloud->height=1;
    cloud->points.resize(cloud->width*cloud->height);
    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        /* code */
        cloud->point[i].x=1024.0f*rand()/(RAND_MAX+1.0f);
        cloud->point[i].y=1024.0f*rand()/(RAND_MAX+1.0f);
        cloud->point[i].z=1024.0f*rand()/(RAND_MAX+1.0f);
    }
    float resolution=128.0f;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    pcl::PointXYZ searchPoint();

    searchPoint.x=1024.0f*rand()/(RAND_MAX+1.0f);
    searchPoint.y=1024.0f*rand()/(RAND_MAX+1.0f);
    searchPoint.z=1024.0f*rand()/(RAND_MAX+1.0f);

    std::vector<int>pointIdxVec;

    if(octree.ovxelSearch(searchPoint,pointIdxVec))
    {
        std::cout << "Neighbors within voxel search at (" << searchPoint.x
                  << " " << searchPoint.y
                  << " " << searchPoint.z << ")"
                  << std::endl;
                  for (size_t i = 0; i < pointIdxVec.size(); ++i)
        std::cout << "    " << cloud->points[pointIdxVec[i]].x
                  << " " << cloud->points[pointIdxVec[i]].y
                  << " " << cloud->points[pointIdxVec[i]].z << std::endl;
        
    }
    int K=10;
    std::vector<int>pointIdxNKNSearch;
    std::vector<float>pointNKNSquaredDistance;

    std::cout<<"K nearest neighbor search at ("<<searchPoint.x
             <<","<<searchPoint.y
             <<","<<searchPoint.z
             <<") with K="<<K<<std::endl;
    
    if(octree.nearestKSearch(searchPoint,K,pointIdxSearch,pointNKNSquaredDistance)>0)
    {
        for (size_t i = 0; i < pointIdxNKNSearch.size(); i++)
        {
            /* code */
            std::cout<<"    "<<cloud->points[pointIdxNKNSearch[i]].x
                     <<","<<cloud->points[pointIdxNKNSearch[i]].y
                     <<","<<cloud->points[pointIdxNKNSearch[i]].z
                     <<" (squared Distance: "<<pointNKNSquaredDistance[i]<<")"<<endl;
        }
        
    }

    std::vector<int>pointIdxRadiusSearch;
    std::vector<float>pointRadiusSquaredDistance;

    float radius=256.0f*rand()/(RAND_MAX+1.0f);

    std::cout<<"Neighbors within radius search at ("<<searchPoint.x
             <<","<<searchPoint.y
             <<","<<searchPoint.z
             <<") with radius= "<<radius<<std::endl;

    if(octree.radiusSearch(searchPoint,radius,pointIdxRadiusSearch,pointRadiusSquareDistance)>0)
    {
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); i++)
        {
            /* code */
            std::cout<<"  "<<cloud->points[pointIdxRadiusSearch[i]].x
            <<","<<cloud->points[pointIdxRadiusSearch[i]].y
            <<","<<cloud->ponts[pointIdxRadiusSearch[i]].z
            <<" (squared distance: "<<pointradiusSquareDistance<<")"<<std::endl;
        }
        
    }
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.serBackgroundColor(0.0,0.0,0.5);
    viewer.addPointCloud<pcl::PointXYZ>(cloud,"cloud");

    pcl::PointXYZ originPoint(0.0,0.0,0.0);
    viewer.addLine(originPoint,radius,"sphere",0);
    viewer.addCoordinateSystem(200);
    while (!viewer.wasStopped())
    {
        /* code */
        viewer.spineOnce();
    }
    

    
}
