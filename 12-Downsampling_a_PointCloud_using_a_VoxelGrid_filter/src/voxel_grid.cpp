#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>


int main (int argc, char** argv)
{
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr filtered_cloud (new pcl::PCLPointCloud2 ());

    // pcl viewer
//    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D viewer"));

    // Fill in the cloud data
    pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    reader.read ("../table_scene_lms400.pcd", *cloud); // Remember to download the file first!

    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
       << " data points (" << pcl::getFieldsList (*cloud) << ").";
//    // Show point cloud
//    viewer->addPointCloud<pcl::PointCloud<PCLPointCloud2>> (cloud, "Before filtering");
//    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Before filtering");

    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*filtered_cloud);

    std::cerr << "PointCloud after filtering: " << filtered_cloud->width * filtered_cloud->height
       << " data points (" << pcl::getFieldsList (*filtered_cloud) << ").";
//    // Show cloud
//    viewer->addPointCloud<pcl::PCLPointCloud2> (filtered_cloud, "After filtering");
//    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "After filtering");

    pcl::PCDWriter writer;
    writer.write ("../table_scene_lms400_downsampled.pcd", *filtered_cloud,
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);


//    while (!viewer->wasStopped ())
//    {
//        viewer->spinOnce (100);
//    }

    return 0;
}
