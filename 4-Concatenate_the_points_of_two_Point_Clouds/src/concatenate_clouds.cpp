#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::cout << "Usage: " << argv[0] << " -f or -p " << std::endl;
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZ> cloud_a, cloud_b, cloud_c;
    pcl::PointCloud<pcl::Normal> n_cloud_b;
    pcl::PointCloud<pcl::PointNormal> p_n_cloud_c;
    //
    cloud_a.width = 5;
    cloud_a.height = 1;
    cloud_a.points.resize (cloud_a.width * cloud_a.height);
    //
    for ( size_t i = 0; i < cloud_a.size(); ++i)
    {
        cloud_a.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud_a.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud_a.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }
    std::cout << "Cloud A:\n";
    for (size_t i = 0; i < cloud_a.size(); ++i)
        std::cout << " " << cloud_a.points[i].x
                  << " " << cloud_a.points[i].y
                  << " " << cloud_a.points[i].z << std::endl;

    if (strcmp(argv[1],"-p") == 0)
    {
        // Size
        cloud_b.width = 3;
        cloud_b.height = 1;
        cloud_b.points.resize(cloud_b.width * cloud_b.height);
        // Assignment value
        for ( size_t i = 0; i < cloud_b.size(); ++i)
        {
            cloud_b.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
            cloud_b.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
            cloud_b.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
        }
        // Print cloud B
        std::cout << "Cloud B:\n";
        for (size_t i = 0; i < cloud_b.size(); ++i)
            std::cout << " " << cloud_b.points[i].x
                      << " " << cloud_b.points[i].y
                      << " " << cloud_b.points[i].z << std::endl;
        // Copy the point cloud data
        cloud_c = cloud_a;
        cloud_c += cloud_b;
        // Print cloud C
        std::cout << "Cloud C:\n";
        for (size_t i = 0; i < cloud_c.size(); ++i)
            std::cout << " " << cloud_c.points[i].x
                      << " " << cloud_c.points[i].y
                      << " " << cloud_c.points[i].z << std::endl;
    }
    else
    {
        // Size
        n_cloud_b.width = 5;
        n_cloud_b.height = 1;
        n_cloud_b.points.resize(n_cloud_b.width * n_cloud_b.height);
        // Assignment value
        for ( size_t i = 0; i < n_cloud_b.size(); ++i)
        {
            n_cloud_b.points[i].normal[0] = 1024 * rand() / (RAND_MAX + 1.0f);
            n_cloud_b.points[i].normal[1] = 1024 * rand() / (RAND_MAX + 1.0f);
            n_cloud_b.points[i].normal[2] = 1024 * rand() / (RAND_MAX + 1.0f);
        }
        // Print B
        std::cout << "Cloud B:\n";
        for (size_t i = 0; i < n_cloud_b.size(); ++i)
        {
            std::cout << " " << n_cloud_b.points[i].normal[0]
                      << " " << n_cloud_b.points[i].normal[1]
                      << " " << n_cloud_b.points[i].normal[2] << std::endl;
        }
        // Copy the cloud data
        pcl::concatenateFields(cloud_a, n_cloud_b, p_n_cloud_c);
        std::cout << "Cloud C:\n";
        for (size_t i = 0; i < p_n_cloud_c.size(); ++i)
        {
            std::cout << " " << p_n_cloud_c.points[i].x
                      << " " << p_n_cloud_c.points[i].y
                      << " " << p_n_cloud_c.points[i].z
                      << " " << p_n_cloud_c.points[i].normal[0]
                      << " " << p_n_cloud_c.points[i].normal[1]
                      << " " << p_n_cloud_c.points[i].normal[2] << std::endl;
        }
    }




    return 0;
}
