#include <iostream>
#include <iomanip>
#include <experimental/filesystem>
#include <regex>
#include <algorithm>
// PCL header
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/console/time.h> // TicToc
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
// radius outlier remove.
#include <pcl/filters/radius_outlier_removal.h>

// CPD headers
#include <cpd/jsoncpp.hpp>
#include <cpd/rigid.hpp>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::Normal PointNT;
typedef pcl::PointCloud<PointNT> CloudNormalT;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointNormalCloudT;

bool next_iteration = false;

void print4x4Matrix(const Eigen::Matrix4d &matrix)
{
    printf("Rotation matrix :\n");
    printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
    printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
    printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
    printf("Translation vector :\n");
    printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
                           void *nothing)
{
    if (event.getKeySym() == "space" && event.keyDown())
        next_iteration = true;
}

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        // std::cout << "Usage: " << argv[0] << "rail_std.pcd pcd_dir iter" << std::endl;
        pcl::console::print_error("Usage: %s %s\n", argv[0], "rail_std.pcd pcd_dir iter");
        return -1;
    }
    int iterations = 0; // Default number of ICP iterations
    if (argc > 3)
    {
        // If the user passed the number of iteration as an argument
        iterations = atoi(argv[3]);
        if (iterations < 1)
        {
            PCL_ERROR("Number of initial iterations must be >= 1\n");
            return (-1);
        }
    }

    PointCloudT::Ptr std_cloud_ptr(new PointCloudT());
    PointCloudT::Ptr gocator_cloud_ptr(new PointCloudT());
    PointCloudT::Ptr icp_std_cloud_ptr(new PointCloudT());
    PointCloudT::Ptr icp_gocator_cloud_ptr(new PointCloudT());
    PointCloudT::Ptr trans_gocator_cloud_ptr(new PointCloudT());

    if (pcl::io::loadPCDFile(argv[1], *std_cloud_ptr) == -1)
    {
        PCL_ERROR("Cound't load rail std cloud %s\n", argv[1]);
        return -1;
    }
    // Read gocator pcd files.
    std::vector<std::string> pcd_vec;
    std::string pcd_dir = std::string(argv[2]);
    if (std::experimental::filesystem::is_directory(pcd_dir))
    {
        std::regex pcd_file(".*.*\\.(pcd)"); //包含字母z的所有jpg或png图片
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

    // if(pcl::io::loadPCDFile (pcd_vec[0], *gocator_cloud_ptr)==-1)
    // {
    //     PCL_ERROR("Cound't load rail gocator cloud %s\n",argv[2]);
    //     return -1;
    // }

    // Defining a rotation matrix and translation vector
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(std_cloud_ptr);
    sor.setMeanK(30);
    sor.setStddevMulThresh(3.0);
    sor.filter(*icp_std_cloud_ptr);
    // Passthrough
    pcl::PassThrough<PointT> pass;
    // All the objects needed
    pass.setInputCloud(icp_std_cloud_ptr);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.0, 1.0);
    pass.filter(*icp_std_cloud_ptr);
    pass.setInputCloud(icp_std_cloud_ptr);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(0.002, 0.18); // laser_0: [0.008, 0.15], laser_3: [0.004, 0.15]
    pass.filter(*icp_std_cloud_ptr);
    pass.setInputCloud(icp_std_cloud_ptr);
    // pass.setFilterFieldName("y");
    // pass.setFilterLimits(0.10, 0.14);
    // pass.setFilterLimitsNegative(true); // return the data outside the interval specified by setFilterLimits (min, max)
    // pass.filter(*icp_std_cloud_ptr);
    // pass.setFilterLimitsNegative(false);
    // Create the filtering object

    // CPD data prepare.
    cpd::Matrix fixed(icp_std_cloud_ptr->size(), 3);
    // Feed data.
    for (size_t i = 0; i < fixed.rows(); i++)
    {
        fixed(i, 0) = icp_std_cloud_ptr->points[i].x;
        fixed(i, 1) = icp_std_cloud_ptr->points[i].y;
        fixed(i, 2) = 0;
    }

    /////////////////////////////
    pcl::console::TicToc time;

    double total_time = 0.0, score = 1.0;

    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
    int v1(0);
    int v2(1);
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

    // viewer.setBackgroundColor (255, 255, 255);
    // Origin point cloud is white.
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_h(std_cloud_ptr, 255, 255, 255);

    viewer.addPointCloud(std_cloud_ptr, cloud_color_h, "std_cloud", v1);
    viewer.addPointCloud(icp_std_cloud_ptr, cloud_color_h, "icp_std_cloud", v2);

    // ICP cloud in red.
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_icp_h(gocator_cloud_ptr, 200, 0, 0);
    viewer.addPointCloud(gocator_cloud_ptr, cloud_color_icp_h, "gocator_cloud", v1);
    viewer.addPointCloud(icp_gocator_cloud_ptr, cloud_color_h, "icp_gocator_cloud", v2);

    // Add text.
    // addText (const std::string &text, int xpos, int ypos, int fontsize, double r, double g, double b, const std::string &id="", int viewport=0)
    viewer.addText("White: Original point cloud", 10, 15, 16, 1.0, 1.0, 1.0, "icp_info_1", v1);
    viewer.addText("Red: ICP aligned point cloud", 10, 0, 16, 1.0, 0.0, 0.0, "icp_info_2", v1);

    std::stringstream ss;
    ss << iterations;
    std::string iterations_cnt = "ICP iterations = " + ss.str();
    viewer.addText(iterations_cnt, 10, 60, 16, 1.0, 1.0, 1.0, "iterations_cnt", v2);
    // Toatal time
    std::string time_str = "Total time: " + std::to_string(total_time) + " ms.";
    viewer.addText(time_str, 10, 40, 16, 1.0, 1.0, 1.0, "total_time", v2);
    // Score
    // score = icp.getFitnessScore();
    std::string score_str = "ICP Score: " + std::to_string(score);
    viewer.addText(score_str, 10, 20, 16, 1.0, 1.0, 1.0, "icp_score", v2);
    // filename
    viewer.addText(pcd_vec[0], 10, 80, 16, 1.0, 1.0, 1.0, "pcd_file", v2);
    // Register keyboard callback :
    viewer.registerKeyboardCallback(&keyboardEventOccurred, (void *)NULL);

    // Radius outlier remover.
    pcl::RadiusOutlierRemoval<PointT> radius_filter;

    // CPD
    cpd::Rigid rigid;    // Rigid transform.
    rigid.scale(false);  // No scale.
    rigid.outliers(0.6); // outliers ratio.
    // rigid.normalize(true); // normalization.
    rigid.max_iterations(150);
    rigid.tolerance(1e-07); //
    /** In 'cpd/src/rigid.cpp'
         if (!m_reflections) {
            c(cols - 1, cols - 1) =
                (svd.matrixU() * svd.matrixV().transpose()).determinant();
        }
    **/
    rigid.reflections(false); // Sets whether this rigid transform allows reflections.
    cpd::RigidResult result;

    int cnt = 0;
    // double score = 1.0;
    while (!viewer.wasStopped() && cnt < pcd_vec.size())
    {
        viewer.spinOnce(200);
        if (next_iteration)
        {
            transformation_matrix = Eigen::Matrix4d::Identity();
            // Read gocator cloud.
            if (pcl::io::loadPCDFile(pcd_vec[cnt], *gocator_cloud_ptr) == -1)
            {
                PCL_ERROR("Cound't load rail gocator cloud %s\n", argv[2]);
                return -1;
            }
            // // Scale
            // for (size_t i = 0; i < gocator_cloud_ptr->size(); i++)
            // {
            //     gocator_cloud_ptr->points[i].x *= 1000.0;
            //     gocator_cloud_ptr->points[i].y *= 1000.0;
            // }
            viewer.updateText(pcd_vec[cnt], 10, 80, 16, 1.0, 1.0, 1.0, "pcd_file");
            // gocator cloud
            // sor.setInputCloud(gocator_cloud_ptr);
            // sor.setMeanK(30);
            // sor.setStddevMulThresh(4.0);
            // sor.filter(*gocator_cloud_ptr);
            // Radius filter.
            radius_filter.setInputCloud(gocator_cloud_ptr);
            radius_filter.setRadiusSearch(0.005); // 10 mm
            radius_filter.setMinNeighborsInRadius(3);
            radius_filter.filter(*icp_gocator_cloud_ptr);
            // gocator cloud.
            pass.setFilterLimitsNegative(false);
            pass.setInputCloud(icp_gocator_cloud_ptr);
            pass.setFilterFieldName("x");
            pass.setFilterLimits(-1.0, 1.0); // gocator_0: [0.01,10]; 3:[0.004, 1.0]
            pass.filter(*icp_gocator_cloud_ptr);
            // pass.setInputCloud (gocator_cloud_ptr);
            // pass.setFilterFieldName ("y");
            // pass.setFilterLimits (0.04, 1.0);
            // pass.filter (*icp_gocator_cloud_ptr);
            // pcl::io::savePCDFileASCII("filter_gocator.pcd", *icp_gocator_cloud_ptr);
            // std::cout << "Saved " << icp_gocator_cloud_ptr->points.size() << " points to rail_std_pcd.pcd." << std::endl;
            // CPD data.
            cpd::Matrix moving(icp_gocator_cloud_ptr->size(), 3);
            for (size_t i = 0; i < moving.rows(); i++)
            {
                moving(i, 0) = icp_gocator_cloud_ptr->points[i].x;
                moving(i, 1) = icp_gocator_cloud_ptr->points[i].y;
                moving(i, 2) = 0;
            }
            // The Iterative Closest Point algorithm
            viewer.spinOnce(100);
            time.tic();
            // icp.Matrix4;
            result = rigid.run(fixed, moving);
            // result = rigid.run(moving, fixed);
            std::cout << cpd::to_json(result) << std::endl;
            // Copy to trans matrix.
            transformation_matrix.block(0, 0, 3, 3) = result.rotation;
            transformation_matrix.block(0, 3, 3, 1) = result.translation;

            total_time += time.toc();
            std::cout << "Applied 1 ICP iteration in " << time.toc() << " ms" << std::endl;

            print4x4Matrix(transformation_matrix); // Print the transformation between original pose and current pose
            // Rotation angle.
            double angle = std::atan2(transformation_matrix(0, 1), transformation_matrix(0, 0));
            std::cout << "Rotation angle: " << std::fixed << std::setprecision(6) << angle << std::endl;
            // Iteration.
            ss.str("");
            ss << result.iterations;
            std::string iterations_cnt = "ICP iterations = " + ss.str();
            viewer.updateText(iterations_cnt, 10, 60, 16, 1.0, 1.0, 1.0, "iterations_cnt");
            // Time
            std::string time_str = "Total time: " + std::to_string(total_time) + " ms.";
            viewer.updateText(time_str, 10, 40, 16, 1.0, 1.0, 1.0, "total_time");
            // score = icp.getFitnessScore();
            ss.str("");
            ss << "Score: " << std::scientific << std::setprecision(1) << result.sigma2;
            viewer.updateText(ss.str(), 10, 20, 16, 1.0, 1.0, 1.0, "icp_score");
            pcl::transformPointCloud<PointT, double>(*icp_gocator_cloud_ptr, *icp_gocator_cloud_ptr, transformation_matrix);
            // pcl::transformPointCloud<PointT, double>(*icp_gocator_cloud_ptr, *icp_gocator_cloud_ptr, transformation_matrix.inverse());
            viewer.updatePointCloud<PointT>(icp_gocator_cloud_ptr, cloud_color_icp_h, "icp_gocator_cloud");

            // Transform gocator cloud.
            pcl::transformPointCloud<PointT, double>(*gocator_cloud_ptr, *trans_gocator_cloud_ptr, transformation_matrix);
            // pcl::transformPointCloud<PointT, double>(*gocator_cloud_ptr, *trans_gocator_cloud_ptr, transformation_matrix.inverse());
            viewer.updatePointCloud(trans_gocator_cloud_ptr, cloud_color_icp_h, "gocator_cloud");
            // Reset.
            next_iteration = false;
            cnt++;
            total_time = 0.0;
        }
    }
    viewer.spin();
    return 0;
}
