#include <iostream>

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// This cpp file custom
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/normal_3d.h>


int main(int argc, char** argv)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    cloud->width  = 5;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);

    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }

    std::cerr << "Cloud before projection: " << std::endl;
    for (size_t i = 0; i < cloud->points.size (); ++i)
        std::cerr << "    " << cloud->points[i].x << " "
                            << cloud->points[i].y << " "
                            << cloud->points[i].z << std::endl;


    //Create planar coefficients with X=Y=0,Z=1,d=0
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    //  ax + by + cz + d = 0, where a=b=d=0, and c=1 , the X-Y plane
    coefficients->values.resize(4);
    coefficients->values[0] = coefficients->values[1] = 0;
    coefficients->values[2] = 1;
    coefficients->values[3] = 0;

    // filter
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_LINE);     // SACMODEL_LINE,SACMODEL_CIRCLE2D,
    proj.setInputCloud(cloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloud_projected);

    // Show out the projected cloud
    // Note that their z now lies on the X-Y plane.
    std::cout << "After projected cloud:\n";
    for(int i = 0; i < cloud_projected->size(); ++i)
    {
        std::cerr << "  " << cloud_projected->points[i].x
                  << "  " << cloud_projected->points[i].y
                  << "  " << cloud_projected->points[i].z << std::endl;

    }


    return 0;
}
