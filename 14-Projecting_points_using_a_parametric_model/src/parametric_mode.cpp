#include <iostream>

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// This cpp file custom
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char **argv)
{

    if (argc < 2)
    {
        PCL_ERROR("Usage: %s 1.pcd\n", argv[0]);
        return -1;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::visualization::PCLVisualizer viewer("cloud viewer");
    /**
     *  Read cloud instead.
     * **/
    if (pcl::io::loadPCDFile(argv[1], *cloud) == -1)
    {
        PCL_ERROR("Cound't load rail std cloud %s\n", argv[1]);
        return -1;
    }
    // Show.
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud, 255, 0, 0);
    viewer.addPointCloud(cloud, color, "raw_cloud", 0);

    //Create Line coefficients with Z=0
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    //  ax + by + cz + d = 0, where c = 0;
    // coefficients->values.resize(4);
    // //    coefficients->values[0] = coefficients->values[1] = 0;
    // coefficients->values[2] = 0;
    // coefficients->values[3] = 0;
    // Circle: (cx, cy, R)
    coefficients->values.resize(3);
    coefficients->values[0] = 0;
    coefficients->values[1] = 0;
    coefficients->values[2] = 0.2;  // 400mm

    // filter
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_CIRCLE2D); // SACMODEL_LINE,SACMODEL_CIRCLE2D,
    proj.setInputCloud(cloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloud_projected);

    // Show out the projected cloud
    // Show.
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_line(cloud_projected, 0, 255, 0);
    viewer.addPointCloud(cloud_projected, color_line, "line_cloud", 0);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce(200);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return 0;
}
