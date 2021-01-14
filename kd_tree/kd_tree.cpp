#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <iostream>
#include <vector>
#include <ctime>

#include <pcl/visualization/cloud_viewer.h>


int main(int argc, char const *argv[])
{
    /* code */
    srand(time(NULL));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width =1000;
    cloud->height=1;
    cloud->points.resize(cloud->width*cloud->height);

    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        /* code */
        cloud->points[i].x=1024.0f*rand()/(RAND_MAX+1.0f);
        cloud->points[i].y=1024.0f*rand()/(RAND_MAX+1.0f);
        cloud->points[i].z=1024.0f*rand()/(RAND_MAX+1.0f);
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

    kdtree.setInputCloud(cloud);

    pcl::PointXYZ searchPoint;
    searchPoint.x=1024.0f*rand()/(RAND_MAX+1.0f);
    searchPoint.y=1024.0f*rand()/(RAND_MAX+1.0f);
    searchPoint.z=1024.0f*rand()/(RAND_MAX+1.0f);



    int K=10;
    std::vector<int>pointIdxNKNSearch(K);
    std::vector<float>pointNKNSquareDistance(K);

    std::cout<<"k nearest neighbor search at("<<
        searchPoint.x<<","<<searchPoint.y<<","<<searchPoint.z
        <<")with k="<<K<<std::endl;
    
    if(kdtree.nearestKSearch(searchPoint,K,pointIdxNKNSearch,pointNKNSquareDistance)>0)
    {
        for (size_t i = 0; i < pointIdxNKNSearch.size(); i++)
        {
            /* code */
            std::cout<<"   "<<cloud->points[pointIdxNKNSearch[i]].x
            <<","<<cloud->points[pointIdxNKNSearch[i]].y<<","<<
            cloud->points[pointIdxNKNSearch[i]].z<<
            "距离平方："<<pointNKNSquareDistance[i]<<std::endl;
        }
        
    }

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    float radius=256.0f*rand()/(RAND_MAX+1.0f);
    std::cout<<"Neighbors within radius search at ("<<
        searchPoint.x<<","<<searchPoint.y<<","<<searchPoint.z
        <<")with radius= "<<radius<<std::endl;


     if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
            std::cout << "    " << cloud->points[pointIdxRadiusSearch[i]].x
                      << "," << cloud->points[pointIdxRadiusSearch[i]].y
                      << "," << cloud->points[pointIdxRadiusSearch[i]].z
                      << " (距离平方:: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
    }


    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(0.0,0.0,0,0);
    viewer.addPointCloud<pcl::PointXYZ>(cloud,"cloud");

    pcl::PointXYZ originPoint(0.0,0.0,0.0);
    viewer.addLine(originPoint,searchPoint);
    viewer.addSphere(searchPoint,radius,"sphere",0);
    viewer.addCoordinateSystem(200);

    while (!viewer.wasStopped())
    {
        /* code */
        viewer.spinOnce();
    }
    
    return 0;
}
