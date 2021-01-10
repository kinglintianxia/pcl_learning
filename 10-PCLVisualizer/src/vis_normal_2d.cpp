#include <iostream>
#include <fstream>
// PCL header
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_plotter.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

struct RailPoint
{
    float x;
    float y;
    RailPoint(const float &px, const float &py)
    {
        x = px;
        y = py;
    }
    // Operator ' << ' reload.
    friend std::ostream &operator<<(std::ostream &str, const RailPoint &pt)
    {
        str << pt.x << ", " << pt.y;
        return str;
    }
};

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " rail_std.csvs" << std::endl;
        return -1;
    }
    // Write pcl cloud data.
    // pcl::PointCloud<pcl::PointXY>::Ptr cloud (new pcl::PointCloud<pcl::PointXY>());
    PointCloudT::Ptr cloud(new PointCloudT());

    std::vector<RailPoint> rail_pts;
    std::vector<std::pair<double, double>> rail_pairs;
    std::string rail_std_file = std::string(argv[1]);
    std::ifstream infile;
    // *.csv file
    if (rail_std_file.find("csv") != std::string::npos)
    {
        infile.open(rail_std_file.c_str());
        if (!infile)
            std::cout << "Rail std file does not exist!" << std::endl;
        // Read rail std points.
        std::string line;

        while (!infile.eof())
        {
            //        std::string line;
            //        getline(infile, line, ',');
            getline(infile, line);
            //        std::cout << line << std::endl;
            // Skip comments.
            if (line[0] != '#' && line[0] != 'P')
            {
                int pos = line.find(",");
                if (pos != -1)
                {
                    float px = std::atof(line.substr(0, pos).c_str());
                    float py = std::atof(line.substr(pos + 1, line.length()).c_str());
                    rail_pts.push_back(RailPoint(px, py));
                    rail_pairs.push_back(std::make_pair<double, double>(px, py));
                }
            }
        }
        cloud->width = rail_pairs.size();
        cloud->height = 1;
        cloud->is_dense = false;
        cloud->resize(cloud->width * cloud->height);

        for (size_t i = 1; i < cloud->size(); i++)
        {
            cloud->points[i].x = rail_pairs[i].first / 1000;
            cloud->points[i].y = rail_pairs[i].second / 1000;
            // cloud->points[i].z = 0;
        }

        pcl::io::savePCDFileASCII("rail_std_pcd.pcd", *cloud);
        std::cout << "Saved " << cloud->points.size() << " points to rail_std_pcd.pcd." << std::endl;
    }
    else if (rail_std_file.find("pcd") != std::string::npos)
    {
        /* code */
        if (pcl::io::loadPCDFile(rail_std_file, *cloud) == -1)
        {
            PCL_ERROR("Cound't load rail std cloud %s\n", argv[1]);
            return -1;
        }
        for (size_t i = 1; i < cloud->size(); i++)
        {
            rail_pairs.push_back(std::make_pair<double, double>(cloud->points[i].x, cloud->points[i].y));
            // cloud->points[i].z = 0;
        }
    }
    else
    {
        std::cout << "Invalid file, Please check!" << std::endl;
        return -1;
    }

    std::cout << "rail_std point num: " << rail_pairs.size() << std::endl;
    // for (auto pt:rail_pts)
    //     std::cout << pt << std::endl;

    pcl::visualization::PCLPlotter *plotter = new pcl::visualization::PCLPlotter();
    plotter->addPlotData(rail_pairs, "rail_std", vtkChart::POINTS);
    plotter->plot();

    return 0;
}