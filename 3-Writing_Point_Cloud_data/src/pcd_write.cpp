#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;

    cloud.width = 5;
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.resize (cloud.width * cloud.height);

    for (size_t i = 0; i < cloud.size (); ++i)
    {
        cloud.points[i].x = 1024 * rand() /(RAND_MAX + 1.0f);
        cloud.points[i].y = 1024 * rand() /(RAND_MAX + 1.0f);
        cloud.points[i].z = 1024 * rand() /(RAND_MAX + 1.0f);
    }

    pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
    std::cout << "Saved " << cloud.points.size () << " points to test_pcd.pcd." << std::endl;

    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
        std::cout << " " << cloud.points[i].x
                  << " " << cloud.points[i].y
                  << " " << cloud.points[i].z << std::endl;
    }

    return 0;

}
