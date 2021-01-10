#include <iostream>
#include <thread>
#include <chrono>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

// using namespace std::chrono_literals;

typedef pcl::PointXYZ PointT;

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        pcl::console::print_error("Usage: %s %s %s", argv[0], "target.pcd", "source.cpd\n");
        exit(1);
    }

    // Loading first scan of room.
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *target_cloud) == -1)
    {
        PCL_ERROR("Couldn't read file room_scan1.pcd \n");
        return (-1);
    }
    // Loading second scan of room from new perspective.
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *input_cloud) == -1)
    {
        PCL_ERROR("Couldn't read file room_scan2.pcd \n");
        return (-1);
    }
    std::cout << "Loaded " << target_cloud->size() << " data points from room_scan1.pcd" << std::endl;

    std::cout << "Loaded " << input_cloud->size() << " data points from room_scan2.pcd" << std::endl;

    // // Filtering input scan to roughly 10% of original size to increase speed of registration.
    // pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    // approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
    // approximate_voxel_filter.setInputCloud(input_cloud);
    // approximate_voxel_filter.filter(*filtered_cloud);
    // std::cout << "Filtered cloud contains " << filtered_cloud->size()
    //           << " data points from room_scan2.pcd" << std::endl;

    // Passthrough
    pcl::PassThrough<PointT> pass;
    // All the objects needed
    pass.setInputCloud(target_cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.0, 1.0);
    pass.filter(*target_cloud);
    pass.setInputCloud(target_cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(0.008, 0.18); // laser_0: [0.008, 0.15], laser_3: [0.004, 0.15]
    pass.filter(*target_cloud);
    // pass.setInputCloud(icp_std_cloud_ptr);
    // pass.setFilterFieldName("y");
    // pass.setFilterLimits(0.10, 0.14);
    // pass.setFilterLimitsNegative(true); // return the data outside the interval specified by setFilterLimits (min, max)
    // pass.filter(*icp_std_cloud_ptr);
    // pass.setFilterLimitsNegative(false);
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(target_cloud);
    sor.setMeanK(20);
    sor.setStddevMulThresh(1.0);
    sor.filter(*target_cloud);

    // Source cloud.
    sor.setInputCloud(input_cloud);
    sor.setMeanK(30);
    sor.setStddevMulThresh(1.0);
    sor.filter(*input_cloud);
    // gocator cloud.
    pass.setFilterLimitsNegative(false);
    pass.setInputCloud(input_cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.005, 1.0); // gocator_0: [0.01,10]; 3:[0.004, 1.0]
    pass.filter(*input_cloud);

    // Initializing Normal Distributions Transform (NDT).
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

    // Setting scale dependent NDT parameters
    // Setting minimum transformation difference for termination condition.
    // ndt.setTransformationEpsilon(0.01);
    ndt.setEuclideanFitnessEpsilon(1e-07);
    // Setting maximum step size for More-Thuente line search.
    // ndt.setStepSize(0.005);
    //Setting Resolution of NDT grid structure (VoxelGridCovariance).
    // ndt.setResolution(0.001);

    // Setting max number of registration iterations.
    ndt.setMaximumIterations(1);

    // Setting point cloud to be aligned.
    ndt.setInputSource(input_cloud);
    // Setting point cloud to be aligned to.
    ndt.setInputTarget(target_cloud);

    // Set initial alignment estimate found using robot odometry.
    // Eigen::AngleAxisf init_rotation(-31.0296255187058/180.0 * 3.1415926, Eigen::Vector3f::UnitZ());
    // Eigen::Translation3f init_translation(-0.002495686150609759, 0.115705700461929, 0);
    // Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();
    Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
    double angle = 31.0296255187058/180.0 * 3.1415926;
    init_guess(0, 0) = std::cos(angle);
    init_guess(0, 1) = sin(angle);
    init_guess(1, 0) = -sin(angle);
    init_guess(1, 1) = std::cos(angle);
    init_guess(0, 3) = -0.002495686150609759;
    init_guess(1, 3) = 0.115705700461929;
    
    // Calculating required rigid transform to align the input cloud to the target cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    ndt.align(*output_cloud, init_guess);

    std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
              << " score: " << ndt.getFitnessScore() << std::endl;

    // Transforming unfiltered, input cloud using found transform.
    pcl::transformPointCloud(*input_cloud, *output_cloud, ndt.getFinalTransformation());
    // output_cloud = input_cloud;
    // Saving transformed input cloud.
    // pcl::io::savePCDFileASCII("room_scan2_transformed.pcd", *output_cloud);

    // Initializing point cloud visualizer
    pcl::visualization::PCLVisualizer::Ptr
        viewer_final(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer_final->setBackgroundColor(0, 0, 0);

    // Coloring and visualizing target cloud (red).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        target_color(target_cloud, 255, 0, 0);
    viewer_final->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                   2, "target cloud");

    // Coloring and visualizing transformed input cloud (green).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        output_color(output_cloud, 0, 255, 0);
    viewer_final->addPointCloud<pcl::PointXYZ>(output_cloud, output_color, "output cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                   2, "output cloud");

    // Starting visualizer
    // viewer_final->addCoordinateSystem(1.0, "global");
    viewer_final->initCameraParameters();

    // Wait until visualizer window is closed.
    while (!viewer_final->wasStopped())
    {
        viewer_final->spinOnce(100);
        // std::this_thread::sleep_for(100);
    }

    return (0);
}