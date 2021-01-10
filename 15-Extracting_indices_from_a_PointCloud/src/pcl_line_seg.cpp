#include <iostream>
#include <experimental/filesystem>
#include <iomanip>
#include <regex>
#include <algorithm>
#include <cmath>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
// radius outlier remove.
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/time.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
bool next_iteration = false;

// cloud pts.
struct Point
{
	double x;
	double y;
	Point()
	{
		x = 0;
		y = 0;
	}
	// Constrcutor
	Point(double px, double py)
		: x(px), y(py)
	{
	}
	// Copy constructor.
	Point(const Point &pt)
	{
		x = pt.x;
		y = pt.y;
	}
	// Operator "=" reload.
	void operator =(const Point &pt)
	{
		x = pt.x;
		y = pt.y;
	}
	// Operator '-' reload
	Point operator -(const Point &pt) const
	{
		return Point(x - pt.x, y - pt.y);
	}
	// Operator '+' reload
	Point operator +(const Point &pt) const
	{
		return Point(x + pt.x, y + pt.y);
	}
};

// Calibration std params.
const double calib_outer_width = 1611 / 1000.0;
const double calib_inner_width = 1405 / 1000.0;
const double calib_std_width = (calib_outer_width - calib_inner_width) / 2.0;
// std Point.
const double calib_li_pt_x = 0;
const double calib_lo_pt_x = -calib_std_width;
const double calib_ri_pt_x = calib_inner_width;
const double calib_ro_pt_x = calib_inner_width + calib_std_width;
// laser3 flag.
bool laser3_flag = false;
Point calib_li_pt;

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
						   void *nothing)
{
	if (event.getKeySym() == "space" && event.keyDown())
		next_iteration = true;
}

// Get line param 'a, b, c', ax+by+c=0
void get_line_param(const Point &pt1, const Point &pt2, double &a, double &b, double &c)
{
	a = pt1.y - pt2.y;
	b = pt2.x - pt1.x;
	c = pt1.x * pt2.y - pt2.x * pt1.y;
};

// cacl intersection point of two lines.
bool caclInterPoint(const pcl::ModelCoefficients &line1, const pcl::ModelCoefficients &line2, Point &inter_pt)
{
	// Line prams.
	double a1, b1, c1, a2, b2, c2, d;
	// [point_on_line.x point_on_line.y point_on_line.z line_direction.x line_direction.y line_direction.z]
	get_line_param(Point(line1.values[0], line1.values[1]), Point(line1.values[0] + line1.values[3], line1.values[1] + line1.values[4]), a1, b1, c1);
	get_line_param(Point(line2.values[0], line2.values[1]), Point(line2.values[0] + line2.values[3], line2.values[1] + line2.values[4]), a2, b2, c2);
	// Parallell?
	// a1/a2 = b1/b2 -> a1*b2-a2*b1 = 0
	d = a1 * b2 - a2 * b1;
	if (std::abs(d) < 1e-6)
		return false;
	// Get intersection point.
	inter_pt.x = (b1 * c2 - b2 * c1) / d;
	inter_pt.y = (c1 * a2 - c2 * a1) / d;
	return true;
}

// // Get the Signed angle.
// double getSignedAngle(const Point &pta, const Point &ptb, const Point &pt_common)
// {
// 	// Get Line vector.
// 	Point vector_a, vector_b;
// 	vector_a.x = pta.x - pt_common.x;
// 	vector_a.y = pta.y - pt_common.y;
// 	vector_b.x = ptb.x - pt_common.x;
// 	vector_b.y = ptb.y - pt_common.y;
// 	// Angle vector_a to vector_b
// 	// angle_b minus angle_a.
// 	return (std::atan2(vector_b.y, vector_b.x) - std::atan2(vector_a.y, vector_a.x));
// }

// Get the Signed angle. (-pi,pi]
double getSignedAngle(const Point &vector_a, const Point &vector_b)
{
	// Get Line vector.
	// cos<a,b>, get abs(angle)
	double a_dot_b = vector_a.x * vector_b.x + vector_a.y * vector_b.y;
	// sin<a,b>, get the sign of angle.
	double a_mul_b = vector_a.x * vector_b.y - vector_a.y * vector_b.x;
	return std::atan2(a_mul_b, a_dot_b);
}
// Get translation
bool getTranslation(int laser_id, const Point &inter_pt, Point &trans)
{
	bool ret = false;
	switch (laser_id)
	{
	case 0 /* constant-expression */:
		/* code */
		if (laser3_flag)
		{
			Point calib_ro_pt(calib_inner_width + calib_std_width, 0);
			// t = t2+t1
			trans = calib_ro_pt + (calib_li_pt-inter_pt);
			// trans = (calib_li_pt-inter_pt);
			ret = true;
		}
		break;
	case 1 /* constant-expression */:
		/* code */
		if (laser3_flag)
		{
			Point calib_ri_pt(calib_inner_width, 0);
			trans = calib_ri_pt + (calib_li_pt-inter_pt);
			// trans = (calib_li_pt-inter_pt);
			ret = true;
		}
		break;
	case 2 /* constant-expression */:
		/* code */
		if (laser3_flag)
		{
			Point calib_lo_pt(-calib_std_width, 0);
			trans = calib_lo_pt + (calib_li_pt-inter_pt);
			// trans = (calib_li_pt-inter_pt);
			ret = true;
		}
		break;
	case 3 /* constant-expression */:
		/* code */
		calib_li_pt = inter_pt;
		trans = Point(0.0, 0.0);
		laser3_flag = true;
		ret = true;
		break;

	default:
		break;
	}
	return ret;
}

int main(int argc, char **argv)
{
	if (argc < 2)
	{
		// std::cout << "Usage: " << argv[0] << "rail_std.pcd pcd_dir iter" << std::endl;
		pcl::console::print_error("Usage: %s \n", argv[0], "pcd_dir");
		return -1;
	}

	PointCloudT::Ptr cloud_ptr(new PointCloudT());
	PointCloudT::Ptr cloud_filtered_ptr(new PointCloudT());
	PointCloudT::Ptr cloud_pos(new PointCloudT()), cloud_neg(new PointCloudT());
	PointCloudT::Ptr cloud_trans(new PointCloudT());
	// Viewer.
	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	int v1(0);
	int v2(1);
	int v3(2);
	viewer.createViewPort(0.0, 0.0, 0.5, 0.5, v1);
	viewer.createViewPort(0.5, 0.0, 1.0, 0.5, v2);
	viewer.createViewPort(0.0, 0.5, 1.0, 1.0, v3);
	// viewer.setBackgroundColor (255, 255, 255);

	// Origin point cloud is white.
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_h(cloud_ptr, 255, 255, 255);
	viewer.addPointCloud(cloud_ptr, cloud_color_h, "src_cloud", v1);

	// ICP cloud in red.
	pcl::visualization::PointCloudColorHandlerCustom<PointT> filtered_cloud_color_h(cloud_filtered_ptr, 200, 0, 0);
	viewer.addPointCloud(cloud_filtered_ptr, filtered_cloud_color_h, "filtered_cloud", v1);
	// Register keyboard callback :
	viewer.registerKeyboardCallback(&keyboardEventOccurred, (void *)NULL);

	// Read gocator pcd files.
	std::vector<std::string> pcd_vec;
	std::string pcd_dir = std::string(argv[1]);
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
	if (pcd_vec.empty())
	{
		std::cerr << "No pcd file in dir!" << std::endl;
		return -1;
	}
	std::sort(pcd_vec.begin(), pcd_vec.end());
	// filename
	viewer.addText(pcd_vec[0], 10, 80, 16, 1.0, 1.0, 1.0, "pcd_file", v2);
	// Convert to the templated PointCloud
	// pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Radius outlier remover.
	pcl::RadiusOutlierRemoval<PointT> radius_filter;
	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	// SACMODEL_LINE - used to determine line models. The six coefficients of the line are given by a point on the line and the direction of the line as: [point_on_line.x point_on_line.y point_on_line.z line_direction.x line_direction.y line_direction.z]
	seg.setModelType(pcl::SACMODEL_LINE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(0.001); // 2mm
	// seg.setAxis(Eigen::Vector3f(0, 0, 1));
	// seg.setProbability(0.8);
	seg.setRadiusLimits(0.01, 0.05); // 1cm-5cm

	// coefficients
	// coefficients->values.resize(3);
	// Store params for lines.
	std::vector<pcl::ModelCoefficients> line_coefficents_vec;
	std::vector<pcl::PointIndices> line_indices_vec;
	Point inter_pt;

	int cnt = pcd_vec.size() - 1;
	std::stringstream ss;
	// double score = 1.0;
	while (!viewer.wasStopped() && cnt >= 0) // from laser3 to laser0
	{
		viewer.spinOnce(200);
		if (next_iteration)
		{
			viewer.removeAllPointClouds(v2);
			line_coefficents_vec.clear();
			line_indices_vec.clear();

			pcl::ScopeTime time("fitting lines");
			// Read gocator cloud.
			if (pcl::io::loadPCDFile(pcd_vec[cnt], *cloud_ptr) == -1)
			{
				PCL_ERROR("Cound't load rail gocator cloud %s\n", argv[2]);
				return -1;
			}
			viewer.updateText(pcd_vec[cnt], 10, 80, 16, 1.0, 1.0, 1.0, "pcd_file");
			viewer.updatePointCloud<PointT>(cloud_ptr, cloud_color_h, "src_cloud");
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "src_cloud", v1);
			// Laser id.
			std::stringstream cvt;
			int laser_id = 0;
			cvt.str("");
			cvt << pcd_vec[cnt][pcd_vec[cnt].size() - 7];
			cvt >> laser_id;

			// Radius filter.
			radius_filter.setInputCloud(cloud_ptr);
			radius_filter.setRadiusSearch(0.010); // 10 mm
			radius_filter.setMinNeighborsInRadius(5);
			radius_filter.filter(*cloud_ptr);

			int i = 0, nr_points = (int)cloud_ptr->points.size();
			pcl::copyPointCloud<PointT>(*cloud_ptr, *cloud_filtered_ptr);
			// While 30% of the original cloud is still there
			while (cloud_filtered_ptr->points.size() > 10)
			{
				// Segment the largest planar component from the remaining cloud
				seg.setInputCloud(cloud_filtered_ptr);
				seg.segment(*inliers, *coefficients);
				if (inliers->indices.size() == 0)
				{
					std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
					break;
				}

				// Extract the inliers
				extract.setInputCloud(cloud_filtered_ptr);
				extract.setIndices(inliers);
				extract.setNegative(false);
				extract.filter(*cloud_pos);
				std::cerr << "cloud_" << cnt << "_line_" << i + 1 << " has: " << cloud_pos->width * cloud_pos->height << " data points,";
				// Line coefficients.
				std::cout << " coefficients: " << *coefficients << std::endl;
				ss.str("");
				ss << cnt << "_line_" << i;

				// Origin point cloud is white.
				std::srand(i * 50);
				pcl::visualization::PointCloudColorHandlerCustom<PointT> line_color_h(cloud_pos, 255, (255 - 50 * i), (std::rand() % 255 - i * 100));
				viewer.addPointCloud(cloud_pos, line_color_h, ss.str(), v2);

				// Store current model coefficents.
				line_coefficents_vec.push_back(*coefficients);
				// Store current indices.
				line_indices_vec.push_back(*inliers);

				// Create the filtering object
				// outliers
				extract.setNegative(true);
				extract.filter(*cloud_neg);
				cloud_filtered_ptr.swap(cloud_neg);
				// viewer.updatePointCloud<PointT>(cloud_filtered_ptr, filtered_cloud_color_h, "filtered_cloud");

				i++;
			}

			// Get inter pt.
			// Select line segment.
			double max_y = -1e6;
			int select_line_index = 0;
			std::vector<int> vertical_line_index;
			for (size_t i = 0; i < line_coefficents_vec.size(); i++)
			{
				double current_y = line_coefficents_vec[i].values[1];
				if (current_y > max_y)
				{
					max_y = current_y;
					select_line_index = i;
				}
			}
			// cacl vertical intersection points.
			std::vector<Point> pts_vec;
			for (size_t i = 0; i < line_coefficents_vec.size(); i++)
			{
				Point pt;
				// Check vertical line's inter pt.
				// vector a*b = 0;
				// x1*x2 + y1*y2 = 0
				double zero = (line_coefficents_vec[i].values[3] * line_coefficents_vec[select_line_index].values[3] + line_coefficents_vec[i].values[4] * line_coefficents_vec[select_line_index].values[4]);
				if (zero < 5e-02)
				{
					bool result = caclInterPoint(line_coefficents_vec[select_line_index], line_coefficents_vec[i], pt);
					if (result)
					{
						vertical_line_index.push_back(i);
						pts_vec.push_back(pt);
					}
				}
			}
			if (!pts_vec.empty())
			{
				// Find the intersection point that have maximum y axis value.
				// std::sort(pts_vec.begin(), pts_vec.end(), [](Point &pt1, Point &pt2) { return (pt1.y > pt2.y); });
				double max_y = -1e6;
				int vertical_index = 0;
				for (size_t i = 0; i < pts_vec.size(); i++)
				{
					if (pts_vec[i].y > max_y)
					{
						max_y = pts_vec[i].y;
						vertical_index = i;
					}
				}
				// Draw inter point.
				inter_pt = pts_vec[vertical_index];
				pcl::ModelCoefficients circle_coeff;
				circle_coeff.values.resize(3); // We need 3 values
				circle_coeff.values[0] = inter_pt.x;
				circle_coeff.values[1] = inter_pt.y;
				circle_coeff.values[2] = 0.003;
				ss.str("");
				ss << cnt << "inter_pt";
				viewer.addCircle(circle_coeff, ss.str(), v2); // (x, y, radius)

				// Highlight selected lines.
				std::cout << "Selected lines index: " << select_line_index << ", " << vertical_line_index[vertical_index] << std::endl;
				// Cacl rotation angle & translation vector.
				double angle = 0.0;
				Point choose_pt;
				Point pt1(line_coefficents_vec[select_line_index].values[0], line_coefficents_vec[select_line_index].values[1]);
				Point pt2(line_coefficents_vec[vertical_line_index[vertical_index]].values[0], line_coefficents_vec[vertical_line_index[vertical_index]].values[1]);
				if (laser_id == 0 || laser_id == 3)
				{
					// The line has minimum x.
					if (pt1.x < pt2.x)
					{
						std::cout << "Angle line: " << select_line_index << std::endl;
						// Choosen pt.
						choose_pt = pt1;
					}
					else
					{
						std::cout << "Angle line: " << vertical_line_index[vertical_index] << std::endl;
						choose_pt = pt2;
					}
					// Draw line pt.
					circle_coeff.values[0] = choose_pt.x;
					circle_coeff.values[1] = choose_pt.y;
					circle_coeff.values[2] = 0.003;
					ss.str("");
					ss << cnt << "line_pt";
					viewer.addCircle(circle_coeff, ss.str(), v2); // (x, y, radius)
					// Cacl Signed angle.
					angle = getSignedAngle(Point(-1, 0), choose_pt - inter_pt);
					// Get translation, inter_pt
				}
				else // laser1 & laser2
				{
					// The line has maximum x.
					if (pt1.x > pt2.x)
					{
						std::cout << "Angle line: " << select_line_index << std::endl;
						// Choosen pt.
						choose_pt = pt1;
					}
					else
					{
						std::cout << "Angle line: " << vertical_line_index[vertical_index] << std::endl;
						choose_pt = pt2;
					}
					// Draw line pt.s
					circle_coeff.values[0] = choose_pt.x;
					circle_coeff.values[1] = choose_pt.y;
					circle_coeff.values[2] = 0.003;
					ss.str("");
					ss << cnt << "line_pt";
					viewer.addCircle(circle_coeff, ss.str(), v2); // (x, y, radius)
					// Get signed angle.
					angle = getSignedAngle(Point(1, 0), choose_pt - inter_pt);
				}
				std::cout << "Laser " << laser_id << " angle(rad): " << angle << ", deg: " << angle * 180 / 3.1415926 << std::endl;
				// Get translation vector.
				Point trans_vector;
				// Rotate 'inter_pt' first.
				Point temp_pt = inter_pt;
				inter_pt.x = std::cos(angle)*temp_pt.x + std::sin(angle)*temp_pt.y;
				inter_pt.y = -std::sin(angle)*temp_pt.x + std::cos(angle)*temp_pt.y;
				bool trans_flag = getTranslation(laser_id, inter_pt, trans_vector);
				if (trans_flag) // Draw trans cloud.
				{
					std::cout << "Laser " << laser_id << " translation vector: [ " << trans_vector.x << "," << trans_vector.y << " ]" << std::endl;
					// Transform cloud.
					Eigen::Matrix4d trans_mat = Eigen::Matrix4d::Identity();
					trans_mat(0, 0) = std::cos(angle);
					trans_mat(0, 1) = std::sin(angle);
					trans_mat(1, 0) = -std::sin(angle);
					trans_mat(1, 1) = std::cos(angle);
					trans_mat(0, 3) = trans_vector.x;
					trans_mat(1, 3) = trans_vector.y;
					pcl::transformPointCloud<PointT, double>(*cloud_ptr, *cloud_trans, trans_mat);
					ss.str("");
					ss << cnt << laser_id;
					pcl::visualization::PointCloudColorHandlerCustom<PointT> trans_color_h(cloud_trans, 255, (255 - 50 * laser_id), (std::rand() % 255 - i * 100));
					viewer.addPointCloud(cloud_trans, trans_color_h, ss.str(), v3);
					// Draw laser3 center point.
					circle_coeff.values[0] = 0;
					circle_coeff.values[1] = 0;
					circle_coeff.values[2] = 0.003;
					ss.str("");
					ss << cnt << "center_pt";
					viewer.addCircle(circle_coeff, ss.str(), v3); // (x, y, radius)
				}
			}
			viewer.spinOnce();
			// Reset.
			next_iteration = false;
			cnt--;
		}
		// viewer.removeAllPointClouds();
	}
	viewer.spin();
	return (0);
}