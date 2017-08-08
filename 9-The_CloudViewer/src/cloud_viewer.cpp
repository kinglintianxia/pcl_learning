#include <iostream>
// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>

int user_data;

void viewerOnceOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (1.0, 0.5, 1.0);
    pcl::PointXYZ point;
    point.x = 1.0;
    point.y = 0;
    point.z = 0;
    viewer.addSphere (point, 0.25, "sphere", 0);
    std::cout << "I only run once." << std::endl;


}

void viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned cnt = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << cnt++;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str (), 200, 300, 16, 1, 0, 0, "text", 0);
    user_data++;

}

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    if (argc != 2)
        pcl::io::loadPCDFile ("../map.pcd", *cloud);
    else
        pcl::io::loadPCDFile (argv[1], *cloud);

    pcl::visualization::CloudViewer viewer("Cloud Viewer");

    viewer.showCloud (cloud);

    viewer.runOnVisualizationThreadOnce (viewerOnceOff);
    viewer.runOnVisualizationThread (viewerPsycho);

    while (!viewer.wasStopped ())
    {
        user_data++;
    }


    return 0;
}
