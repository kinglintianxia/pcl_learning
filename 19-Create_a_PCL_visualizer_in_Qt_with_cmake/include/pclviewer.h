#ifndef PCLVIEWER_H
#define PCLVIEWER_H

#include <iostream>

// Qt
#include <QMainWindow>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// We declare the namespace Ui and the class PCLViewer inside it.
namespace Ui
{
  class PCLViewer;
}

class PCLViewer : public QMainWindow
{
  // the macro Q_OBJECT tells the compiler that this object contains UI elements
  Q_OBJECT

public:
  // the keyword 'explicit' to suppress automatic conversions
  // invoked by one-argument constructors; C++ primer P1174
  explicit PCLViewer (QWidget *parent = 0);
  ~PCLViewer ();

// 插槽
public Q_SLOTS:
// these functions will be linked with UI elements actions.
  void
  randomButtonPressed ();

  void
  RGBsliderReleased ();

  void
  pSliderValueChanged (int value);

  void
  redSliderValueChanged (int value);

  void
  greenSliderValueChanged (int value);

  void
  blueSliderValueChanged (int value);

protected:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  PointCloudT::Ptr cloud;

  unsigned int red;
  unsigned int green;
  unsigned int blue;

private:
  Ui::PCLViewer *ui;

};

#endif // PCLVIEWER_H
