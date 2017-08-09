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

    // Fill in the cloud data
    pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    reader.read ("../table_scene_lms400.pcd", *cloud); // Remember to download the file first!

    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
       << " data points (" << pcl::getFieldsList (*cloud) << ").";



    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*filtered_cloud);

    std::cerr << "PointCloud after filtering: " << filtered_cloud->width * filtered_cloud->height
       << " data points (" << pcl::getFieldsList (*filtered_cloud) << ").";

    pcl::PCDWriter writer;
    writer.write ("../table_scene_lms400_downsampled.pcd", *filtered_cloud,
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);


    return 0;
}
