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
// normal_estimate2d
#include <normal_estimate2d/Normal2dEstimation.h>
// normal regictor
#include <pcl/registration/correspondence_rejection_surface_normal.h>
// sample_consensus_2d
#include <pcl/registration/correspondence_rejection_sample_consensus_2d.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/kdtree/kdtree.h>

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
    // Normal estimation.
    CloudNormalT::Ptr std_cloud_normal(new CloudNormalT());
    CloudNormalT::Ptr gocator_cloud_normal(new CloudNormalT());
    PointNormalCloudT::Ptr icp_std_normal_ptr(new PointNormalCloudT());
    PointNormalCloudT::Ptr icp_gocator_normal_ptr(new PointNormalCloudT());
    PointNormalCloudT::Ptr trans_gocator_normal_cloud_ptr(new PointNormalCloudT());
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    Normal2dEstimation norm_estim;

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

    // if(pcl::io::loadPCDFile (pcd_vec[0], *gocator_cloud_ptr)==-1)
    // {
    //     PCL_ERROR("Cound't load rail gocator cloud %s\n",argv[2]);
    //     return -1;
    // }

    // Transformed point cloud is green
    double angle[4] = {-31.0296255187058, 23.8186524404734, 23.0288788583355, -21.6930059944138}; // 四个 2D 激光扫描仪的安装角度
    // Eigen::AngleAxisd rot_vec(angle[0] / 180.0 * 3.1415926, Eigen::Vector3d(0, 0, 1));
    Eigen::Matrix4f track_pose(Eigen::Matrix4f::Identity());
    // Eigen::Matrix<float, 4, 4> track_pose()
    // track_pose.rotate(rot_vec.matrix());
    // track_pose(0, 0) = std::cos(angle[0]);
    // track_pose(0, 1) = sin(angle[0]);
    // track_pose(1, 0) = -sin(angle[0]);
    // track_pose(1, 1) = std::cos(angle[0]);
    //
    track_pose(0, 0) = 0.847;
    track_pose(0, 1) = 0.532;
    track_pose(1, 0) = -0.532;
    track_pose(1, 1) = 0.847;
    //
    // Matrix4 pcl_mat(Matrix4::Identity());

    // Defining a rotation matrix and translation vector
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

    // Passthrough
    pcl::PassThrough<PointT> pass;
    // All the objects needed
    pass.setInputCloud(std_cloud_ptr);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.0, 1.0);
    pass.filter(*icp_std_cloud_ptr);
    pass.setInputCloud(icp_std_cloud_ptr);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(0.005, 0.18); // laser_0: [0.008, 0.15], laser_3: [0.004, 0.15]
    pass.filter(*icp_std_cloud_ptr);
    // // pass.setInputCloud(icp_std_cloud_ptr);
    // pass.setFilterFieldName("y");
    // pass.setFilterLimits(0.10, 0.14);
    // pass.setFilterLimitsNegative(true); // return the data outside the interval specified by setFilterLimits (min, max)
    // pass.filter(*icp_std_cloud_ptr);
    // pass.setFilterLimitsNegative(false);
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(icp_std_cloud_ptr);
    sor.setMeanK(20);
    sor.setStddevMulThresh(4.0);
    sor.filter(*icp_std_cloud_ptr);
    // Normal
    norm_estim.setInputCloud(icp_std_cloud_ptr);
    norm_estim.setSearchMethod(tree);
    norm_estim.setRadiusSearch(0.01); // 10mm
    norm_estim.compute(std_cloud_normal);
    // Concadate.
    pcl::concatenateFields(*icp_std_cloud_ptr, *std_cloud_normal, *icp_std_normal_ptr);

    pcl::console::TicToc time;
    //    time.tic();
    // The Iterative Closest Point algorithm
    // time.tic();
    pcl::IterativeClosestPoint<PointNormalT, PointNormalT> icp;
    // pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> icp;
    // icp.setMaximumIterations(iterations);
    // icp.setInputSource(icp_gocator_cloud_ptr);
    icp.setInputTarget(icp_std_normal_ptr);
    //
    icp.setTransformationEpsilon(1e-3);
    //设置最大近邻点距离，超过此距离的点对将不参与计算
    // icp.setMaxCorrespondenceDistance(0.02);
    // icp.addCorrespondenceRejector();
    // icp.setUseReciprocalCorrespondences(true);

    //设置迭代误差收敛阈值
    icp.setEuclideanFitnessEpsilon(2e-07);
    // icp.align(*icp_gocator_cloud_ptr);
    icp.setMaximumIterations(10); // We set this variable to 1 for the next time we will call .align () function
    

    double total_time = 0.0, score = 1.0;
    // total_time += time.toc();
    // std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc() << " ms" << std::endl;

    // if (icp.hasConverged())
    // {
    //     std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
    //     std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
    //     transformation_matrix = icp.getFinalTransformation().cast<double>();
    //     print4x4Matrix(transformation_matrix);
    // }
    // else
    // {
    //     PCL_ERROR("\nICP has not converged.\n");
    //     return (-1);
    // }

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
    // viewer.addPointCloudNormals<PointT, PointNT>(icp_std_cloud_ptr, std_cloud_normal, 1, 0.005, "icp_std_cloud_normals", v2);
    // ICP cloud in red.
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_icp_h(gocator_cloud_ptr, 200, 0, 0);
    viewer.addPointCloud(gocator_cloud_ptr, cloud_color_icp_h, "gocator_cloud", v1);
    // // ICP cloud in red.
    pcl::visualization::PointCloudColorHandlerCustom<PointNormalT> normal_cloud_color_icp_h(icp_gocator_normal_ptr, 200, 0, 0);
    viewer.addPointCloud(icp_gocator_normal_ptr, normal_cloud_color_icp_h, "icp_gocator_cloud", v2);

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
            sor.setInputCloud(gocator_cloud_ptr);
            sor.setMeanK(30);
            sor.setStddevMulThresh(3.0);
            sor.filter(*gocator_cloud_ptr);
            // gocator cloud.
            pass.setFilterLimitsNegative(false);
            pass.setInputCloud(gocator_cloud_ptr);
            pass.setFilterFieldName("x");
            pass.setFilterLimits(-1.0, 1.0); // gocator_0: [0.01,10]; 3:[0.004, 1.0]
            pass.filter(*icp_gocator_cloud_ptr);
            // pass.setInputCloud (gocator_cloud_ptr);
            // pass.setFilterFieldName ("y");
            // pass.setFilterLimits (-1.0, -0.05);
            // pass.filter (*icp_gocator_cloud_ptr);
            // pcl::io::savePCDFileASCII("filter_gocator.pcd", *icp_gocator_cloud_ptr);
            // std::cout << "Saved " << icp_gocator_cloud_ptr->points.size() << " points to rail_std_pcd.pcd." << std::endl;
            // Normal
            norm_estim.setInputCloud(icp_gocator_cloud_ptr);
            norm_estim.setSearchMethod(tree);
            // Use all neighbors in a sphere of radius 10mm
            norm_estim.setRadiusSearch(0.02);
            norm_estim.compute(gocator_cloud_normal);
            // Concadate.
            pcl::concatenateFields(*icp_gocator_cloud_ptr, *gocator_cloud_normal, *icp_gocator_normal_ptr);

            viewer.updatePointCloud<PointNormalT>(icp_gocator_normal_ptr, normal_cloud_color_icp_h, "icp_gocator_cloud");
            // std::string normal_id = "icp_gocator_cloud_normals" + std::to_string(cnt);viewer.addPointCloudNormals<pcl::PointNormal>(icp_gocator_normal_ptr, 1, 0.01, normal_id, v2);

            icp.setInputSource(icp_gocator_normal_ptr);
            /******  CorrespondenceEstimationNormalShooting   *********/
            // // CorrespondenceEstimationNormalShooting
            // pcl::registration::CorrespondenceEstimationNormalShooting<PointNormalT, PointNormalT, PointNormalT>::Ptr cens(new pcl::registration::CorrespondenceEstimationNormalShooting<PointNormalT, PointNormalT, PointNormalT>);
            // cens->setInputSource(icp_gocator_normal_ptr);
            // cens->setSourceNormals(icp_gocator_normal_ptr);
            // cens->setInputTarget(icp_std_normal_ptr);
            // // cens->setKSearch(10);
            // // std::cout << "--- Need normals: " << cens->requiresTargetNormals() << std::endl;
            // // cens->setTargetNormals(icp_std_normal_ptr);
            // // Draw correspondence.
            // pcl::Correspondences all_correspondences;
            // // Determine all correspondences
            // cens->determineCorrespondences(all_correspondences);

            // for (std::size_t j = 0; j < all_correspondences.size(); ++j)
            // {
            //     // std::stringstream ss_line;
            //     std::string ss_line = std::to_string(cnt) + "_" + std::to_string(j);
            //     PointNormalT &model_point = icp_gocator_normal_ptr->at(all_correspondences[j].index_query);
            //     PointT &scene_point = icp_std_cloud_ptr->at(all_correspondences[j].index_match);
            //     //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
            //     viewer.addLine<PointNormalT, PointT>(model_point, scene_point, 0, 255, 0, ss_line, v2);
            // }
            //
            // icp.setCorrespondenceEstimation(cens);
            // icp.addCorrespondenceRejector(rej);
            // icp.getCorrespondenceRejectors()

            /***********  CorrespondenceRejectorSurfaceNormal   *****************/
            // // CorrespondenceRejectorSurfaceNormal
            // pcl::registration::CorrespondenceRejectorSurfaceNormal::Ptr rej_normal(new pcl::registration::CorrespondenceRejectorSurfaceNormal());
            // // rej_normal->setInputSource<PointNormalT>(icp_gocator_normal_ptr);
            // // // rej_normal->setSourceNormals(icp_gocator_normal_ptr);
            // // rej_normal->setInputTarget<PointNormalT>(icp_std_normal_ptr);
            // // rej_normal->setTargetNormals(icp_std_normal_ptr);
            // rej_normal->setThreshold(0.1); // 45 deg.
            // icp.addCorrespondenceRejector(rej_normal);

            /***********  CorrespondenceRejectorSampleConsensus2D   *****************/
            pcl::registration::CorrespondenceRejectorSampleConsensus<PointNormalT>::Ptr rej_sac2d(new pcl::registration::CorrespondenceRejectorSampleConsensus<PointNormalT>());
            rej_sac2d->setInputSource(icp_gocator_normal_ptr);
            rej_sac2d->setInputTarget(icp_std_normal_ptr);
            icp.addCorrespondenceRejector(rej_sac2d);

            // ICP correspondenceRejectorSampleConsensus methods.
            // std::vector<pcl::registration::CorrespondenceRejector::Ptr> icp_corr_vec;
            // icp_corr_vec = icp.getCorrespondenceRejectors();
            // pcl::console::print_highlight("icp_default_rejectors: %s\n", icp_corr_vec.size());
            // for (auto rej : icp_corr_vec)
            //     std::cout << "Rejector: " << rej->getClassName() << std::endl;


            // The Iterative Closest Point algorithm
            while (iterations < 100 && score > 2.0e-07) // gocator_3: 2.2e-07
            {
                viewer.spinOnce(100);
                time.tic();
                // icp.Matrix4;

                icp.align(*icp_gocator_normal_ptr);
                
                total_time += time.toc();
                std::cout << "Applied 1 ICP iteration in " << time.toc() << " ms" << std::endl;

                if (icp.hasConverged())
                {
                    //                printf("\033[11A"); // Go up 11 lines in terminal output.
                    score = icp.getFitnessScore();
                    std::printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
                    iterations += 10;
                    std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
                    transformation_matrix = icp.getFinalTransformation().cast<double>() * transformation_matrix; // WARNING /!\ This is not accurate! For "educational" purpose only!
                    print4x4Matrix(transformation_matrix);                                                       // Print the transformation between original pose and current pose

                    ss.str("");
                    ss << iterations;
                    std::string iterations_cnt = "ICP iterations = " + ss.str();
                    viewer.updateText(iterations_cnt, 10, 60, 16, 1.0, 1.0, 1.0, "iterations_cnt");
                    std::string time_str = "Total time: " + std::to_string(total_time) + " ms.";
                    viewer.updateText(time_str, 10, 40, 16, 1.0, 1.0, 1.0, "total_time");
                    ss.str("");
                    ss << "ICP Score: " << std::scientific << std::setprecision(1) << icp.getFitnessScore();
                    viewer.updateText(ss.str(), 10, 20, 16, 1.0, 1.0, 1.0, "icp_score");
                    viewer.updatePointCloud<PointNormalT>(icp_gocator_normal_ptr, normal_cloud_color_icp_h, "icp_gocator_cloud");
                }
                else
                {
                    PCL_ERROR("\nICP has not converged.\n");
                    // return -1;
                    break;
                }
                // Transform gocator cloud.
                Eigen::Matrix4d trans_matrix = icp.getFinalTransformation().cast<double>();
                pcl::transformPointCloud(*gocator_cloud_ptr, *trans_gocator_cloud_ptr, transformation_matrix);
                viewer.updatePointCloud(trans_gocator_cloud_ptr, cloud_color_icp_h, "gocator_cloud");
            }
            iterations = 0;
            score = 1.0;
            cnt++;
            total_time = 0.0;
        }

        next_iteration = false;
    }
    viewer.spin();
    return 0;
}
