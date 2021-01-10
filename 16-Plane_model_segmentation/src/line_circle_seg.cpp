#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/passthrough.h>


int main(int argc, char **argv)
{
  if (argc < 2)
  {
    PCL_ERROR("Usage: %s test.pcd\n", argv[0]);
    return -1;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::visualization::PCLVisualizer viewer("cloud viewer");

  if (pcl::io::loadPCDFile(argv[1], *cloud) == -1)
  {
    PCL_ERROR("Cound't load rail gocator cloud %s\n", argv[1]);
    return -1;
  }
  // Show.
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud, 255, 0, 0);
  viewer.addPointCloud(cloud, color, "raw_cloud", 0);
  // Seg
  pcl::PassThrough<pcl::PointXYZ> pass;
  // All the objects needed
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-1.0, 0.0);
  pass.filter(*cloud);

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory 强制性
  seg.setModelType(pcl::SACMODEL_LINE); // SACMODEL_LINE,SACMODEL_CIRCLE2D
  seg.setMethodType(pcl::SAC_RANSAC);
  // distance threshold determines how close a point must be to the model in order to be considered an inlier
  seg.setDistanceThreshold(0.001);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0)
  {
    PCL_ERROR("Could not estimate a planar model for the given dataset.");
    return (-1);
  }
  // Show.
  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_line(indices, 0, 255, 0);
  // viewer.addPointCloud(indices, color_line, "line_cloud", 0);

  // ax + by + cz + d = 0, a, b, c ,d
  std::cerr << "Model coefficients: " << coefficients->values[0] << " "
            << coefficients->values[1] << " "
            << coefficients->values[2] << std::endl;

  viewer.addLine<pcl::PointXYZ>(cloud->points[inliers->indices[0]], cloud->points[inliers->indices[inliers->indices.size()-1]], "line");

  // viewer.addCircle(*coefficients, "text_id", 0); // (x, y, radius)

  std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;
  // for (size_t i = 0; i < inliers->indices.size(); ++i)
  //   std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
  //             << cloud->points[inliers->indices[i]].y << " "
  //             << cloud->points[inliers->indices[i]].z << std::endl;
  while (!viewer.wasStopped())
  {
    viewer.spinOnce(200);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }

  return (0);
}
