#ifndef __ddDrakeWrapper_h
#define __ddDrakeWrapper_h

#include <QObject>
#include <QColor>
#include <QVector>
#include "ddDrakeModel.h"
#include "ddAppConfigure.h"


class DD_APP_EXPORT ddDrakeWrapper : public QObject
{
    Q_OBJECT

public:

  ddDrakeWrapper(QObject* parent=0);
  virtual ~ddDrakeWrapper();

  QVector<double> resolveCenterOfPressure(const ddDrakeModel&  ddModel, 
                                          const QVector<int>& ft_frame_ids, 
                                          const QVector<double> & ft_in, 
                                          const QVector<double> & normal_in, 
                                          const QVector<double> & point_on_contact_plane_in) const;
  double drakeSignedDistanceInsideConvexHull(int num_pts,
                                        const QVector<double>& pts_in, 
                                        const QVector<double> & q_in) const;

protected:

  Q_DISABLE_COPY(ddDrakeWrapper);
};

#endif
