#include "ddLidarPointCloudLCM.h"

#include "ddLCMSubscriber.h"
#include "ddLCMThread.h"

#include <lcmtypes/drake/lcmt_lidar_data.hpp>

#include <vtkCellArray.h>
#include <vtkFloatArray.h>
#include <vtkNew.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

#include <QMutexLocker>

namespace
{
  struct SensorData
  {
    QMap<float, drake::lcmt_lidar_data> data;
    bool lastDirection;
    float lastAngle;
  };

  //---------------------------------------------------------------------------
  int32_t pointCount(drake::lcmt_lidar_data const& data)
  {
    for (auto const& field : data.fields)
    {
      if (field.id == drake::lcmt_lidar_data::POSITION)
        return field.num_values / 3;
    }

    return 0;
  }

  //---------------------------------------------------------------------------
  vtkSmartPointer<vtkFloatArray> createDataArray(
    int fieldId, vtkPolyData* polydata)
  {
    auto const fp = vtkSmartPointer<vtkFloatArray>::New();

    switch (fieldId)
    {
      case drake::lcmt_lidar_data::RANGE:
        fp->SetName("Range");
        fp->SetNumberOfComponents(1);
        break;

      case drake::lcmt_lidar_data::INTENSITY:
        fp->SetName("Intensity");
        fp->SetNumberOfComponents(1);
        break;

      case drake::lcmt_lidar_data::RGB_COLOR:
        fp->SetName("Color");
        fp->SetNumberOfComponents(3);
        break;

      case drake::lcmt_lidar_data::ECHO:
        fp->SetName("Echo");
        fp->SetNumberOfComponents(1);
        break;

      case drake::lcmt_lidar_data::SCAN:
        fp->SetName("Scan");
        fp->SetNumberOfComponents(1);
        break;

      default:
        return {};
    }

    polydata->GetPointData()->AddArray(fp);
    return fp;
  }
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
    auto next = message.scan_angle;

    if (message.scan_direction)
    {
      if (!channelData.lastDirection)
      {
        // Erase from after previous message to end of map
        ++last;
        while (last != channelData.data.end())
          last = channelData.data.erase(last);
      }
      else
      {
        // Erase from previous message to insertion point
        while (last != channelData.data.begin() && (--last).key() < next)
          last = channelData.data.erase(last);
      }
    }
    else
    {
      if (channelData.lastDirection)
      {
        // Erase from start to lesser of {last, next}
        last = channelData.data.begin();
        next = qMin(next, channelData.lastAngle);
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
      }

      // Erase up to insertion point for new message
      while (last != channelData.data.end() && last.key() < next)
        last = channelData.data.erase(last);
    }
  }

  channelData.data.insert(message.scan_angle, message);
  channelData.lastAngle = message.scan_angle;
  channelData.lastDirection = message.scan_direction;
}

//-----------------------------------------------------------------------------
void ddLidarPointCloudLCM::extractPolyData(
  QString const& channel, vtkPolyData* pd) const
{
  auto const currentData = data(channel);

  vtkNew<vtkPoints> points;
  vtkNew<vtkCellArray> verts;
  QHash<int, vtkSmartPointer<vtkFloatArray>> fields;

  auto totalPoints = vtkIdType{0};
  for (auto const& scan : currentData)
    totalPoints += pointCount(scan);

  points->Allocate(totalPoints);
  verts->Allocate(totalPoints);

  pd->SetPoints(points.GetPointer());
  pd->SetVerts(verts.GetPointer());

  for (auto const& scan : currentData)
  {
    for (auto const& field : scan.fields)
    {
      if (field.id == drake::lcmt_lidar_data::POSITION)
      {
        auto const& v = field.values;
        for (int32_t n = 0; n < field.num_values; n += 3)
        {
          auto const pointId =
            points->InsertNextPoint(v[n + 0], v[n + 1], v[n + 2]);
          verts->InsertNextCell(1);
          verts->InsertCellPoint(pointId);
        }
      }
      else
      {
        auto& fp = fields[field.id];
        if (!fp && !(fp = createDataArray(field.id, pd)))
          continue; // Field not known

        for (auto const v : field.values)
          fp->InsertNextValue(v);
      }
    }
  }
}
