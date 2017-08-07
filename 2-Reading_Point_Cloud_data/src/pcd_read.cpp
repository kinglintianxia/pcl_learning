#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile (argv[1], *cloud)==-1)
    {
        PCL_ERROR("Cound't load file %s\n",argv[1]);
        return -1;
    }

    std::cout << "Loaded "
              << cloud->width * cloud->height
              << " points from \"" << argv[1]<< "\" pcd file.\n";
    for (size_t i = 0; i < cloud->size(); ++i)
    {
        std::cout << "  " << cloud->points[i].x
                  << " "  << cloud->points[i].y
                  << " "  << cloud->points[i].z << std::endl;
    }

    return 0;
}
