#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>

int main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D viewer"));

    // Read pcd file
    pcl::PCDReader reader;
    reader.read<pcl::PointXYZRGB> ("../table_scene_lms400.pcd", *cloud);
    std::cout << "\nRead cloud:\n" << *cloud << std::endl;
    // Add source cloud
    int v1(1), v2(2), v3(3);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb1(cloud, 255, 255, 255);
    viewer->createViewPort (0, 0, 0.33, 1.0, v1);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb1,"cloud", v1);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud", v1);

    // Statistical filter
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (5.0);
    sor.setStddevMulThresh (1.0);
    sor.filter (*filtered_cloud);

    // Show filtered cloud
    std::cerr << "Cloud after filtered \n";
    std::cerr << *filtered_cloud << std::endl;

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZRGB> ("../table_scene_lms400_filtered.pcd",*filtered_cloud, false);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb2(filtered_cloud, 0, 255, 0);
    viewer->createViewPort (0.33, 0, 0.66, 1.0, v2);
    viewer->addPointCloud<pcl::PointXYZRGB> (filtered_cloud, rgb2, "filtered cloud", v2);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "filtered cloud", v2);

    sor.setNegative (true);
    sor.filter (*filtered_cloud);
    writer.write<pcl::PointXYZRGB> ("../table_scene_lms400_outliers.pcd", *filtered_cloud,false);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb3(filtered_cloud, 255, 0, 0);
    viewer->createViewPort (0.66, 0, 1.0, 1.0, v3);
    viewer->addPointCloud<pcl::PointXYZRGB> (filtered_cloud, rgb3, "outliers cloud", v3);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "outliers cloud", v3);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce(10);
    }

    return 0;
}
