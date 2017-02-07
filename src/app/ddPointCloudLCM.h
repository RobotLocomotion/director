#ifndef __ddPointCloudLCM_h
#define __ddPointCloudLCM_h

#include <QObject>

#include "ddLCMThread.h"
#include "ddLCMSubscriber.h"
#include "ddAppConfigure.h"

#include <string>
#include <sstream>

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkUnsignedIntArray.h>
#include <vtkFloatArray.h>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core/pointcloud2_t.hpp>
#include <lcmtypes/bot_core/pointcloud_t.hpp>

#include <bot_param/param_client.h>
#include <bot_param/param_util.h>

class DD_APP_EXPORT ddPointCloudLCM : public QObject
{
  Q_OBJECT

public:

  ddPointCloudLCM(QObject* parent=NULL);
  
  void init(ddLCMThread* lcmThread, const QString& botConfigFile);
  qint64 getPointCloudFromPointCloud(vtkPolyData* polyDataRender);

  QStringList getLidarNames() const;
  QString getLidarFriendlyName(const QString& lidarName);
  int getLidarFrequency(const QString& lidarName);
  bool displayLidar(const QString& lidarName);
  QList<int> getLidarIntensity(const QString& lidarName);
  QString getLidarChannelName(const QString& lidarName);
  QString getLidarCoordinateFrame(const QString& lidarName);

protected slots:

  void onPointCloudFrame(const QByteArray& data, const QString& channel);
  void onPointCloud2Frame(const QByteArray& data, const QString& channel);


protected:

  BotParam* mBotParam;

  ddLCMThread* mLCM;

  vtkSmartPointer<vtkPolyData> mPolyData;
  int64_t mUtime;
  QMutex mPolyDataMutex;

};

#endif
