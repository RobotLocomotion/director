#include <director_ros/vtk_conversions.h>

#include <cstdio>
#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/PointCloud2.h>

#include <thread>

#include <PythonQt.h>
#include <ddPythonApp.h>
#include <ddVTKObjectMap.h>


//-----------------------------------------------------------------------------
ddVTKObjectMap* pointcloudMap = 0;

//-----------------------------------------------------------------------------
void runApp(int argc, char** argv)
{
  ddPythonApp app;
  app.init(argc, argv);

  pointcloudMap = new ddVTKObjectMap();
  PythonQt::self()->getMainModule().addVariable("pointcloudMap", QVariant::fromValue((QObject*)pointcloudMap));
  app.start();
  pointcloudMap = nullptr;
  ros::requestShutdown();
}

//-----------------------------------------------------------------------------
class PointcloudSubscriber
{
public:

  PointcloudSubscriber(ros::NodeHandle* nh, const std::string topic_name)
  {
    this->topic_name_ = topic_name;
    this->sub_ = nh->subscribe(topic_name, 1, &PointcloudSubscriber::pointcloud_callback, this);
  }

  void pointcloud_callback(const sensor_msgs::PointCloud2Ptr& msg)
  {
    if (pointcloudMap)
    {
      vtkPolyData* poly_data = ConvertPointCloud2ToVtk(msg);
      pointcloudMap->put(this->topic_name_.c_str(), poly_data);
    }
  }

private:

  ros::Subscriber sub_;
  std::string topic_name_;
};


//-----------------------------------------------------------------------------
int main (int argc, char** argv)
{
  ros::init (argc, argv, "director_node");
  ros::NodeHandle nh;

  PointcloudSubscriber sub1(&nh, "/pointcloud");

  bool ros_on_main_thread = false;
  if (ros_on_main_thread)
  {
    // start director on thread
    std::thread appThread(runApp, argc, argv);

    // spin ros on main thread
    ros::spin();
  }
  else
  {
    // spin ros on thread
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // start director on main thread
    runApp(argc, argv);
  }

  return 0;
}
