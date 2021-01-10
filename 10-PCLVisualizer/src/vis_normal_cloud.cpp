#include <iostream>
#include <iomanip>
#include <experimental/filesystem>
#include <regex>
#include <algorithm>
// PCL header
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/features/normal_3d.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<pcl::PointNormal> PointNormalCloudT;

bool next_iteration = true;

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
                           void *nothing)
{
    if (event.getKeySym() == "space" && event.keyDown())
        next_iteration = true;
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        // std::cout << "Usage: " << argv[0] << "rail_std.pcd pcd_dir iter" << std::endl;
        pcl::console::print_error("Usage: %s %s\n", argv[0], "cloud_dir");
        return -1;
    }

    PointCloudT::Ptr cloud(new PointCloudT());
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>());
    PointNormalCloudT::Ptr normal_cloud(new pcl::PointCloud<pcl::PointNormal>());
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::visualization::PCLVisualizer viewer("cloud viewer");

    // Read gocator pcd files.
    std::vector<std::string> pcd_vec;
    std::string pcd_dir = std::string(argv[1]);
    if (std::experimental::filesystem::is_directory(pcd_dir))
    {
        std::regex pcd_file(".*0.*\\.(pcd)"); //包含字母z的所有jpg或png图片
        for (auto &fe : std::experimental::filesystem::directory_iterator(pcd_dir))
        {
            auto fp = fe.path();
            //std::wcout << fp.filename().wstring() << std::endl;
            auto temp = fp.filename();
            if (std::regex_match(temp.string(), pcd_file))
            {
                pcd_vec.push_back(pcd_dir + temp.string());
                // std::cout << temp << std::endl;
                // std::cout << "pcd_vec.size: " << pcd_vec.size() << std::endl;
            }
        }
    }
    std::sort(pcd_vec.begin(), pcd_vec.end());

    int cnt = 0;
    // filename
    viewer.addText(pcd_vec[0], 10, 80, 16, 1.0, 1.0, 1.0, "pcd_file", 0);
    // Register keyboard callback :
    viewer.registerKeyboardCallback(&keyboardEventOccurred, (void *)NULL);

    while (!viewer.wasStopped() && cnt < pcd_vec.size())
    {
        // Clear
        // 
        viewer.spinOnce(200);
        if (next_iteration)
        {
            viewer.removeAllPointClouds();
            viewer.updateText(pcd_vec[cnt], 10, 80, 16, 1.0, 1.0, 1.0, "pcd_file");
            if (pcl::io::loadPCDFile(pcd_vec[cnt], *cloud) == -1)
            {
                PCL_ERROR("Cound't load rail gocator cloud %s\n", argv[2]);
                return -1;
            }
            std::cout << "Read cloud: " << pcd_vec[cnt] << std::endl; 

            // Estimate point normals
            ne.setInputCloud(cloud);
            ne.setRadiusSearch(0.005);
            // ne.setKSearch(5);
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
            ne.setSearchMethod(tree);
            ne.compute(*cloud_normal);
            // Copy the cloud data
            pcl::concatenateFields(*cloud, *cloud_normal, *normal_cloud);
            // Show.
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> color(normal_cloud, 255, 0, 0);
            std::string cloud_id = "normal_cloud" + std::to_string(cnt);
            viewer.addPointCloud(normal_cloud, color, cloud_id, 0);
            viewer.addPointCloudNormals<pcl::PointNormal>(normal_cloud, 1, 0.02, cloud_id + "normals", 0);
            cnt++;
        }

        // viewer.spinOnce(200);
        next_iteration = false;
        
    }
    viewer.spin();
}
