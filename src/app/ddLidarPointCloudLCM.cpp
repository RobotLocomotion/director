#include "ddLidarPointCloudLCM.h"

#include "ddLCMSubscriber.h"
#include "ddLCMThread.h"

#include <lcmtypes/drake/lcmt_lidar_data.hpp>

#include <vtkCellArray.h>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>

#include <QMutexLocker>

namespace
{
  struct SensorData
  {
    QMap<float, drake::lcmt_lidar_data> data;
    float lastAngle;
  };
}

//-----------------------------------------------------------------------------
class ddLidarPointCloudLCMPrivate
{
public:
  ddLCMThread* lcm;
  QHash<QString, SensorData> data;
  QMutex mutex;
};

//-----------------------------------------------------------------------------
ddLidarPointCloudLCM::ddLidarPointCloudLCM(QObject* parent)
  : QObject{parent}, mD{new ddLidarPointCloudLCMPrivate}
{
}

//-----------------------------------------------------------------------------
ddLidarPointCloudLCM::~ddLidarPointCloudLCM()
{
}

//-----------------------------------------------------------------------------
void ddLidarPointCloudLCM::init(ddLCMThread* lcmThread, QString const& channel)
{
  mD->lcm = lcmThread;

  auto const subscriber = new ddLCMSubscriber(channel, this);
  subscriber->setNotifyAllMessagesEnabled(true);

  connect(subscriber, SIGNAL(messageReceived(QByteArray, QString)),
          this, SLOT(addData(QByteArray, QString)),
          Qt::DirectConnection);

  mD->lcm->addSubscriber(subscriber);
}

//-----------------------------------------------------------------------------
QStringList ddLidarPointCloudLCM::channels() const
{
  QMutexLocker lock(&mD->mutex);
  return mD->data.keys();
}

//-----------------------------------------------------------------------------
QList<drake::lcmt_lidar_data> ddLidarPointCloudLCM::data(
  QString const& channel) const
{
  QMutexLocker lock(&mD->mutex);
  return mD->data.value(channel).data.values();
}

//-----------------------------------------------------------------------------
void ddLidarPointCloudLCM::addData(
  QByteArray const& data, QString const& channel)
{
  drake::lcmt_lidar_data message;
  message.decode(data.data(), 0, data.size());

  QMutexLocker lock(&mD->mutex);

  auto& channelData = mD->data[channel];
  if (channelData.data.isEmpty())
  {
    emit channelAdded(channel);
  }
  else
  {
    auto last = channelData.data.find(channelData.lastAngle);
    auto const next = message.scan_angle;

    if (message.scan_direction)
    {
      // TODO
    }
    else
    {
      // Start erasing after previous message
      ++last;

      if (next < channelData.lastAngle)
      {
        // Erase from after previous message to end of map
        while (last != channelData.data.end())
          last = channelData.data.erase(last);

        // Continue erasing from start of map
        last = channelData.data.begin();
      }

      // Erase up to insertion point for new message
      while (last != channelData.data.end() && last.key() < next)
        last = channelData.data.erase(last);
    }
  }

  channelData.data.insert(message.scan_angle, message);
  channelData.lastAngle = message.scan_angle;
}

//-----------------------------------------------------------------------------
void ddLidarPointCloudLCM::extractPolyData(
  QString const& channel, vtkPolyData* pd) const
{
  auto const currentData = data(channel);

  vtkNew<vtkPoints> points;
  vtkNew<vtkCellArray> verts;

  auto totalPoints = vtkIdType{0};
  for (auto const& scan : currentData)
    totalPoints += scan.num_points;

  points->Allocate(totalPoints);
  verts->Allocate(totalPoints);

  pd->SetPoints(points.GetPointer());
  pd->SetVerts(verts.GetPointer());

  for (auto const& scan : currentData)
  {
    for (auto const& p : scan.points)
    {
      auto const pointId = points->InsertNextPoint(p.x, p.y, p.z);
      verts->InsertNextCell(1);
      verts->InsertCellPoint(pointId);
      // TODO field data
    }
  }
}
