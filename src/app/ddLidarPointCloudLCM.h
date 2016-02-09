#ifndef __ddLidarPointCloudLCM_h
#define __ddLidarPointCloudLCM_h

#include "ddAppConfigure.h"

#include <QObject>

namespace drake
{
  class lcmt_lidar_data;
}

class vtkPolyData;

class ddLCMThread;

class ddLidarPointCloudLCMPrivate;

/// Helper class to aggregate data from a LIDAR stream.
///
/// This class collects data from one or more LIDAR data streams and aggregates
/// it into a set of "recent" data, according to an invalidation policy. Per
/// the policy, data is discarded based on the scan angle, when said angle
/// falls within the range covered by consecutive recent data packets. For
/// example, if the current collection includes scan angles \c 1, \c 2, \c 4,
/// \c 6 and \c 8, with \c 1 being the most recently seen scan angle, and a new
/// packet with scan angle \c 3 arrives, the old data with scan angle \c 2 will
/// be discarded because it falls within the range \c 1 to \c 3. This policy
/// accomodates both rotating sensors and bidirectional sweeping sensors.
class DD_APP_EXPORT ddLidarPointCloudLCM : public QObject
{
  Q_OBJECT

public:
  ddLidarPointCloudLCM(QObject* parent = nullptr);
  ~ddLidarPointCloudLCM();

  void init(ddLCMThread* lcmThread,
            QString const& channel = "DRAKE_POINTCLOUD_LIDAR_.*");

  /// Return a list of all known channels.
  QStringList channels() const;

  /// Return all "current" data for the specified channel
  ///
  /// This function returns the collection of the most recent sensor data for
  /// the specified channel, where "most recent" is determined by the data
  /// aggregation invalidation policy. Refer to the class description for
  /// details.
  QList<drake::lcmt_lidar_data> data(QString const& channel) const;

  /// Extract "current" data for the specified channel into a vtkPolyData
  ///
  /// This function "fills" a vtkPolyData with the collection of the most
  /// recent sensor data for the specified channel. The vtkPolyData should be
  /// empty, as this function will reset its points and vertices to new
  /// objects. Non-positional data is stored as ancillary data arrays.
  ///
  /// \sa data(QString const&)
  void extractPolyData(QString const&, vtkPolyData*) const;

signals:
  /// Emitted when a new channel is seen for the first time.
  /// \sa channels()
  void channelAdded(QString);

protected slots:
  void addData(QByteArray const& data, QString const& channel);

protected:
  QScopedPointer<ddLidarPointCloudLCMPrivate> const mD;
};

#endif
