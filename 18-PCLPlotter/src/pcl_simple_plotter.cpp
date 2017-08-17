#include<vector>
#include<iostream>
#include<utility>

#include<pcl/visualization/pcl_plotter.h>


using namespace std;

int main (int argc, char** argv)
{
  //defining a plotter
  pcl::visualization::PCLPlotter * plotter = new pcl::visualization::PCLPlotter ();

  //defining the polynomial function, y = x^2. Index of x^2 is 1, rest is 0
  // 从x的零次项系数开始……，所以vector应为{0, 0, 1}
  vector<double> func1 (3,0); // length: 3; default value: 0
  func1[2] = 1;

//  plotter->setColorScheme(pcl::visualization::vtkColorSeries::WARM);
  //adding the polynomial func1 to the plotter with [-10, 10] as the range in X axis and "y = x^2" as title
  plotter->addPlotData (func1, -10, 10, "y = x^2");
  plotter->setTitle ("simple_plotter");
  plotter->setYTitle ("y");
  plotter->setXTitle ("x");
  //display the plot, DONE!
  plotter->plot ();

//  // ‘Plotting’ Histogram
//  plotter->addHistogramData (func1, 10, "Histogram");

  return 0;
}
