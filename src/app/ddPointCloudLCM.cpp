#include "ddPointCloudLCM.h"

#include <vtkIdTypeArray.h>
#include <vtkCellArray.h>
#include <vtkNew.h>

#include <multisense_utils/conversions_lcm.hpp>

namespace pcl
{
  // Euclidean Velodyne coordinate, including intensity and ring number. 
  struct PointXYZIR
  {
    PCL_ADD_POINT4D;                    // quad-word XYZ
    float    intensity;                 ///< laser intensity reading
    uint16_t ring;                      ///< laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
  } EIGEN_ALIGN16;

};

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZIR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring))


//-----------------------------------------------------------------------------
ddPointCloudLCM::ddPointCloudLCM(QObject* parent) : QObject(parent)
{
  mBotParam = 0;

  mPolyData = vtkSmartPointer<vtkPolyData>::New();
}


//-----------------------------------------------------------------------------
void ddPointCloudLCM::init(ddLCMThread* lcmThread, const QString& botConfigFile)
{

  if (botConfigFile.length()) {
    mBotParam = bot_param_new_from_file(botConfigFile.toLocal8Bit().data());
  } else {
    while (!mBotParam) {
      mBotParam = bot_param_new_from_server(
          lcmThread->lcmHandle()->getUnderlyingLCM(), 0);
    }
  }
  
  mLCM = lcmThread;

  QString channelName = "POINTCLOUD";
  ddLCMSubscriber* subscriber = new ddLCMSubscriber(channelName, this);
  this->connect(subscriber, SIGNAL(messageReceived(const QByteArray&, const QString&)),
          SLOT(onPointCloudFrame(const QByteArray&, const QString&)), Qt::DirectConnection);
  mLCM->addSubscriber(subscriber);

  QString channelName2 = "VELODYNE";
  ddLCMSubscriber* subscriber2 = new ddLCMSubscriber(channelName2, this);
  this->connect(subscriber2, SIGNAL(messageReceived(const QByteArray&, const QString&)),
          SLOT(onPointCloud2Frame(const QByteArray&, const QString&)), Qt::DirectConnection);
  mLCM->addSubscriber(subscriber2);

}



namespace {



//----------------------------------------------------------------------------
vtkSmartPointer<vtkCellArray> NewVertexCells(vtkIdType numberOfVerts)
{
  vtkNew<vtkIdTypeArray> cells;
  cells->SetNumberOfValues(numberOfVerts*2);
  vtkIdType* ids = cells->GetPointer(0);
  for (vtkIdType i = 0; i < numberOfVerts; ++i)
    {
    ids[i*2] = 1;
    ids[i*2+1] = i;
    }

  vtkSmartPointer<vtkCellArray> cellArray = vtkSmartPointer<vtkCellArray>::New();
  cellArray->SetCells(numberOfVerts, cells.GetPointer());
  return cellArray;
}



//----------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData> PolyDataFromPointCloud2Message(pcl::PointCloud<pcl::PointXYZIR>::ConstPtr cloud)
{
  vtkIdType nr_points = cloud->points.size();

  vtkNew<vtkPoints> points;
  points->SetDataTypeToFloat();
  points->SetNumberOfPoints(nr_points);

  vtkNew<vtkFloatArray> intensityArray;
  intensityArray->SetName("intensity");
  intensityArray->SetNumberOfComponents(1);
  intensityArray->SetNumberOfValues(nr_points);

  vtkNew<vtkUnsignedIntArray> ringArray;
  ringArray->SetName("ring");
  ringArray->SetNumberOfComponents(1);
  ringArray->SetNumberOfValues(nr_points);

  vtkIdType j = 0;    
  for (vtkIdType i = 0; i < nr_points; ++i)
  {
    // Check if the point is invalid
    if (!pcl_isfinite (cloud->points[i].x) ||
        !pcl_isfinite (cloud->points[i].y) ||
        !pcl_isfinite (cloud->points[i].z))
      continue;

    float point[3] = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
    points->SetPoint(j, point);

    float intensity = cloud->points[i].intensity;
    intensityArray->SetValue(j,intensity);

    uint16_t ring = cloud->points[i].ring;
    ringArray->SetValue(j,ring);

    j++;
  }
  nr_points = j;
  points->SetNumberOfPoints(nr_points);

  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  polyData->SetPoints(points.GetPointer());
  polyData->GetPointData()->AddArray(intensityArray.GetPointer());
  polyData->GetPointData()->AddArray(ringArray.GetPointer());
  polyData->SetVerts(NewVertexCells(nr_points));
  return polyData;
}


//----------------------------------------------------------------------------

void unpackColor(float f, unsigned char color[]) {
    color[2] = floor(f / 256.0 / 256.0);
    color[1] = floor((f - color[2] * 256.0 * 256.0) / 256.0);
    color[0] = floor(f - color[2] * 256.0 * 256.0 - color[1] * 256.0);
}


vtkSmartPointer<vtkPolyData> PolyDataFromPointCloudMessage(bot_core::pointcloud_t msg)
{

  // Copy over XYZ data
  vtkIdType nr_points = msg.points.size();
  vtkNew<vtkPoints> points;
  points->SetDataTypeToFloat();
  points->SetNumberOfPoints(nr_points);
  for (vtkIdType i = 0; i < nr_points; ++i)
  {
    float point[3] = {msg.points[i][0], msg.points[i][1], msg.points[i][2]};
    points->SetPoint(i, point);
  }
  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  polyData->SetPoints(points.GetPointer());

  // Per channel attributes
  for (size_t j=0; j<msg.channel_names.size(); j++)
  {
    if (msg.channel_names[j] == "rgb_colors" )
    {
       vtkNew<vtkUnsignedCharArray> rgbArray;
       rgbArray->SetName("rgb_colors");
       rgbArray->SetNumberOfComponents(3);
       rgbArray->SetNumberOfTuples(nr_points);

       for (vtkIdType i = 0; i < nr_points; ++i)
       {
         unsigned char color[3];
         unpackColor(msg.channels[j][i], color);
         rgbArray->SetTupleValue(i, color);
       }
       rgbArray->SetNumberOfTuples(nr_points);
       polyData->GetPointData()->AddArray(rgbArray.GetPointer());

    }
    else
    {

      vtkNew<vtkFloatArray> floatArray;
      floatArray->SetName(msg.channel_names[j].c_str() );
      floatArray->SetNumberOfComponents(1);
      floatArray->SetNumberOfValues(nr_points);
      for (vtkIdType i = 0; i < nr_points; ++i)
        floatArray->SetValue(j, (float) msg.channels[j][i] );

      polyData->GetPointData()->AddArray(floatArray.GetPointer());
    }
  }

  polyData->SetVerts(NewVertexCells(nr_points));
  return polyData;
}




};



//-----------------------------------------------------------------------------
void ddPointCloudLCM::onPointCloud2Frame(const QByteArray& data, const QString& channel)
{
  
  bot_core::pointcloud2_t message;
  message.decode(data.data(), 0, data.size());

  //convert to pcl object:
  pcl::PointCloud<pcl::PointXYZIR>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZIR> ());
  pcl::fromLCMPointCloud2( message, *cloud);
  vtkSmartPointer<vtkPolyData> polyData = PolyDataFromPointCloud2Message(cloud);

  QMutexLocker locker(&this->mPolyDataMutex);
  this->mPolyData = polyData;
  this->mUtime = message.utime;
}


//-----------------------------------------------------------------------------
void ddPointCloudLCM::onPointCloudFrame(const QByteArray& data, const QString& channel)
{
  
  bot_core::pointcloud_t message;
  message.decode(data.data(), 0, data.size());

  vtkSmartPointer<vtkPolyData> polyData = PolyDataFromPointCloudMessage(message);

  QMutexLocker locker(&this->mPolyDataMutex);
  this->mPolyData = polyData;
  this->mUtime = message.utime;
}


//-----------------------------------------------------------------------------
qint64 ddPointCloudLCM::getPointCloudFromPointCloud(vtkPolyData* polyDataRender)
{
  QMutexLocker locker(&this->mPolyDataMutex);
  polyDataRender->ShallowCopy(this->mPolyData);
  return this->mUtime;
}

//-----------------------------------------------------------------------------
QStringList ddPointCloudLCM::getLidarNames() const {
  char** lidarNames = bot_param_get_all_planar_lidar_names(mBotParam);

  QStringList names;
  for (int i = 0; lidarNames[i] != 0; ++i) {
    names << lidarNames[i];
  }

  return names;
}

//-----------------------------------------------------------------------------
QString ddPointCloudLCM::getLidarChannelName(const QString& lidarName) {
  QString key = QString("planar_lidars.") + lidarName + QString(".lcm_channel");

  char* channelName;
  if (!bot_param_get_str(mBotParam, key.toLocal8Bit().data(), &channelName) == 0){
    printf("Could not find lcm_channel property of %s\n", lidarName.toLocal8Bit().data());
    return 0;
  }

  return QString(channelName);
}

//-----------------------------------------------------------------------------
QString ddPointCloudLCM::getLidarFriendlyName(const QString& lidarName) {
  QString key = QString("planar_lidars.") + lidarName + QString(".sensor_name");

  char* friendlyName;
  if (!bot_param_get_str(mBotParam, key.toLocal8Bit().data(), &friendlyName) == 0){
    printf("Could not find sensor_name property of %s\n", lidarName.toLocal8Bit().data());
    return 0;
  }

  return QString(friendlyName);
}

//-----------------------------------------------------------------------------
QString ddPointCloudLCM::getLidarCoordinateFrame(const QString& lidarName) {
  QString key = QString("planar_lidars.") + lidarName + QString(".coord_frame");

  char* coordFrame;
  if (!bot_param_get_str(mBotParam, key.toLocal8Bit().data(), &coordFrame) == 0){
    printf("Could not find coord_frame property of %s\n", lidarName.toLocal8Bit().data());
    return 0;
  }

  return QString(coordFrame);
}

//-----------------------------------------------------------------------------
int ddPointCloudLCM::getLidarFrequency(const QString& lidarName) {
  QString key = QString("planar_lidars.") + lidarName + QString(".frequency");

  int frequency = 0;
  if (!bot_param_get_int(mBotParam, key.toLocal8Bit().data(), &frequency) == 0){
    printf("Could not find frequency property of %s\n", lidarName.toLocal8Bit().data());
    return 0;
  }
  return frequency;
}

//-----------------------------------------------------------------------------
bool ddPointCloudLCM::displayLidar(const QString& lidarName) {
  QString key = QString("planar_lidars.") + lidarName + QString(".director_show");

  int displayLidar = 0;
  if (!bot_param_get_boolean(mBotParam, key.toLocal8Bit().data(), &displayLidar) == 0){
    // printf("Could not find display property of %s\n", lidarName.toLocal8Bit().data()); // don't spam
    return false;
  }
  return displayLidar;
}

//-----------------------------------------------------------------------------
QList<int> ddPointCloudLCM::getLidarIntensity(const QString& lidarName) {
  QString key = QString("planar_lidars.") + lidarName + QString(".intensity_range");

  int intensities[2];
  QList<int> qIntensities;
  if(2 != bot_param_get_int_array(mBotParam, key.toLocal8Bit().data(), &intensities[0], 2)) {
    printf("Could not find display property of %s\n", lidarName.toLocal8Bit().data());
    qIntensities.append(0.0);
    qIntensities.append(0.0);
    return qIntensities;
  }

  for (size_t i;i<2;++i) {
    qIntensities.append(intensities[i]);
  }

  return qIntensities;
}