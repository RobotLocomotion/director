#include "ddROSInterface.h"


#include <memory>
#include <ros/master.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <director_ros/vtk_conversions.h>
#include <vtkImageData.h>
#include <vtkPolyData.h>

#include <PythonQt.h>
#include <QImage>

#include <ddVTKObjectMap.h>



// -----------------------------------------------------------------------------
namespace {

// ----------------------------------------------------------------------------
QImage* ConvertCvMatToQImage(cv::Mat* mat) {
  if (mat->empty()) {
    return nullptr;
  }
  return new QImage(mat->data,
                   mat->cols,
                   mat->rows,
                   mat->cols * 3,
                   QImage::Format_RGB888);
}

// ----------------------------------------------------------------------------
std::shared_ptr<cv::Mat> ConvertCvMatToRgb(std::shared_ptr<cv::Mat> mat) {
  if (mat->type() == CV_8UC1) {
    std::shared_ptr<cv::Mat> tmp = std::make_shared<cv::Mat>();
    cvtColor(*mat, *tmp, CV_GRAY2RGB);
    return tmp;
  } else if (mat->type() == CV_8UC3) {
    std::shared_ptr<cv::Mat> tmp = std::make_shared<cv::Mat>();
    cvtColor(*mat, *tmp, CV_BGR2RGB);
    return tmp;
  }
  return nullptr;
}

// ----------------------------------------------------------------------------
vtkImageData* ConvertCvMatToVtk(cv::Mat* mat) {
  if (mat->empty()) {
    return 0;
  }

  int w = mat->cols;
  int h = mat->rows;
  int channels = mat->channels();

  vtkImageData* image = vtkImageData::New();

  int nComponents = channels;
  int componentType = VTK_UNSIGNED_CHAR;

  image->SetDimensions(w, h, 1);
  image->AllocateScalars(componentType, nComponents);

  unsigned char* outPtr = static_cast<unsigned char*>(image->GetScalarPointer(0, 0, 0));
  memcpy(outPtr, mat->data, sizeof(unsigned char)*w*h*nComponents);

  return image;
}

//-----------------------------------------------------------------------------
class ImageSubscriber
{
public:

  ImageSubscriber(ros::NodeHandle* nh, ddVTKObjectMap* object_map, const std::string topic_name)
  {
    this->print_once = true;
    this->topic_name_ = topic_name;
    this->object_map_ = object_map;
    this->sub_ = nh->subscribe(topic_name, 1, &ImageSubscriber::message_callback, this);
  }

  void message_callback(const sensor_msgs::ImagePtr& msg)
  {
    if (this->print_once) {
      this->print_once = false;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    vtkImageData* vtk_image = ConvertCvMatToVtk(&cv_ptr->image);
    double image_timestamp = msg->header.stamp.sec + static_cast<double>(msg->header.stamp.nsec) * 1e9;
    this->object_map_->put(this->topic_name_.c_str(), vtk_image);
  }

private:

  bool print_once;
  ros::Subscriber sub_;
  std::string topic_name_;
  ddVTKObjectMap* object_map_;
};

//-----------------------------------------------------------------------------
class PointcloudSubscriber
{
public:

  PointcloudSubscriber(ros::NodeHandle* nh, ddVTKObjectMap* object_map, const std::string topic_name)
  {
    this->print_once = true;
    this->topic_name_ = topic_name;
    this->object_map_ = object_map;
    this->sub_ = nh->subscribe(topic_name, 1, &PointcloudSubscriber::message_callback, this);
  }

  void message_callback(const sensor_msgs::PointCloud2Ptr& msg)
  {
    if (this->print_once) {
      //PrintPointCloud2MessageInfo(msg);
      this->print_once = false;
    }
    vtkPolyData* poly_data = ConvertPointCloud2ToVtk(msg);
    this->object_map_->put(this->topic_name_.c_str(), poly_data);
  }

private:

  bool print_once;
  ros::Subscriber sub_;
  std::string topic_name_;
  ddVTKObjectMap* object_map_;
};

}  // anonymous namespace


// -----------------------------------------------------------------------------
class ddROSInterface::Internal {
 public:
  Internal() {
    this->initRos();
  }

  void initRos()
  {
    int argc = 1;
    char* argv[] = {const_cast<char*>("dviz-director")};
    ros::init(argc, argv, "director_ros_plugin");
  }

  void initNode()
  {
    this->nh = std::make_unique<ros::NodeHandle>();
    this->startSpinner();
  }

  void addPointcloudSubscriber(const std::string& topic_name)
  {
    this->pointcloud_subs.push_back(
      std::make_unique<PointcloudSubscriber>(this->nh.get(), &this->object_map, topic_name));
  }

  void addImageSubscriber(const std::string& topic_name)
  {
    this->image_subs.push_back(
      std::make_unique<ImageSubscriber>(this->nh.get(), &this->object_map, topic_name));
  }

  void startSpinner()
  {
    if (!this->spinner)
      this->spinner = std::make_unique<ros::AsyncSpinner>(1);
    this->spinner->start();
  }

  void stopSpinner()
  {
    if (this->spinner)
      this->spinner->stop();
  }

  void printTopics()
  {
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
      const ros::master::TopicInfo& info = *it;
      std::cout << "topic_" << it - master_topics.begin() << ": " << info.name << " type: " << info.datatype << std::endl;
    }
  }

  std::unique_ptr<ros::NodeHandle> nh;
  std::unique_ptr<PointcloudSubscriber> sub1;
  std::unique_ptr<ros::AsyncSpinner> spinner;
  std::vector<std::unique_ptr<PointcloudSubscriber>> pointcloud_subs;
  std::vector<std::unique_ptr<ImageSubscriber>> image_subs;

  ddVTKObjectMap object_map;
};


// -----------------------------------------------------------------------------
ddROSInterface::ddROSInterface() : QObject() {
  this->internal_ = new Internal;

}

// -----------------------------------------------------------------------------
ddROSInterface::~ddROSInterface() {
  delete this->internal_;
}

// -----------------------------------------------------------------------------
bool ddROSInterface::rosCheckMaster() {
  return ros::master::check();
}

// -----------------------------------------------------------------------------
void ddROSInterface::rosRequestShutdown() {
  ros::requestShutdown();
}

// -----------------------------------------------------------------------------
void ddROSInterface::addPointcloudSubscriber(const QString& topicName) {
  this->internal_->addPointcloudSubscriber(topicName.toLocal8Bit().data());
}

// -----------------------------------------------------------------------------
void ddROSInterface::addImageSubscriber(const QString& topicName) {
  this->internal_->addImageSubscriber(topicName.toLocal8Bit().data());
}

// -----------------------------------------------------------------------------
void ddROSInterface::initNode() {
  this->internal_->initNode();
}

// -----------------------------------------------------------------------------
void ddROSInterface::startSpinner() {
  this->internal_->startSpinner();
}

// -----------------------------------------------------------------------------
void ddROSInterface::stopSpinner() {
  this->internal_->stopSpinner();
}

// -----------------------------------------------------------------------------
ddVTKObjectMap* ddROSInterface::vtkObjectMap() {
  return &this->internal_->object_map;
}
