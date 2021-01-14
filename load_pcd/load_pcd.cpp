#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>

int 
main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/yingyue/Desktop/PCL_Practice/data/tutorials/ism_test_cat.pcd",*cloud))
    {
        PCL_ERROR("Couldn't read file cat.pcd \n");
        return (-1);
    }
    std::cout << "Loaded"
              << cloud->width*cloud->height
              << " data points from test_pcd.pcd with the following fields: "
              << std ::endl;
    for(size_t i=0;i<cloud->points.size();i++)
    {
        std::cout << "    "<<cloud->points[i].x
                  << "    "<<cloud->points[i].y
                  << "    "<<cloud->points[i].z<<std::endl;
    }
    return 0;
}