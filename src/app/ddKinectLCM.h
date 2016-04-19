#ifndef __ddKinectLCM_h
#define __ddKinectLCM_h

#include <QObject>

#include "ddLCMThread.h"
#include "ddLCMSubscriber.h"
#include "ddAppConfigure.h"


#include <string>
#include <sstream>

#include <lcm/lcm-cpp.hpp>
#include <bot_frames/bot_frames.h>

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkUnsignedCharArray.h>
#include <vtkFloatArray.h>
#include <vtkTransform.h>
#include <vtkMatrix4x4.h>

#include <kinect/kinect-utils.h>

class DD_APP_EXPORT ddKinectLCM : public QObject
{
  Q_OBJECT

public:

  ddKinectLCM(QObject* parent=NULL);
  
  void init(ddLCMThread* lcmThread, const QString& botConfigFile);
  qint64 getPointCloudFromKinect(vtkPolyData* polyDataRender);

protected slots:

  void onKinectFrame(const QByteArray& data, const QString& channel);


protected:

  ddLCMThread* mLCM;

  KinectCalibration* kcal;
  int decimate_;
    
  uint8_t* rgb_buf_ ;

  vtkSmartPointer<vtkPolyData> mPolyData;
  int64_t mUtime;
  QMutex mPolyDataMutex;

};

#endif
