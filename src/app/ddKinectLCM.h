#ifndef __ddKinectLCM_h
#define __ddKinectLCM_h

#include <QObject>
#include <ddMacros.h>

#include "ddLCMThread.h"
#include "ddLCMSubscriber.h"

#include <string>
#include <sstream>

#include <Eigen/Geometry>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core/image_t.hpp>
#include <lcmtypes/bot_core/images_t.hpp>

#include <bot_core/camtrans.h>
#include <bot_param/param_client.h>
#include <bot_param/param_util.h>
#include <bot_frames/bot_frames.h>

#include <image_utils/jpeg.h>

#include <vtkSmartPointer.h>
#include <vtkPNGWriter.h>
#include <vtkImageData.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkUnsignedCharArray.h>
#include <vtkFloatArray.h>
#include <vtkTransform.h>
#include <vtkMatrix4x4.h>

#include <kinect/kinect-utils.h>

class ddKinectLCM : public QObject
 {
  Q_OBJECT

public:

  class CameraData
  {
    public:

    bool mHasCalibration;
    std::string mName;
    std::string mCoordFrame;
    BotCamTrans* mCamTrans;
    bot_core::image_t mImageMessage;
    Eigen::Isometry3d mLocalToCamera;
    Eigen::Isometry3d mBodyToCamera;
    std::vector<uint8_t> mImageBuffer;
    QMutex mMutex;

    CameraData()
    {
      mHasCalibration = false;
      mImageMessage.width = 0;
      mImageMessage.height = 0;
      mImageMessage.utime = 0;
    }

    ~CameraData()
    {
      if (mCamTrans) bot_camtrans_destroy(mCamTrans);
    }
  };

  ddKinectLCM(QObject* parent=NULL);

  virtual ~ddKinectLCM();
  
  void init(ddLCMThread* lcmThread, const QString& botConfigFile);
  void getPointCloudFromKinect(vtkPolyData* polyDataRender);

protected slots:

  // void onImagesMessage(const QByteArray& data, const QString& channel);
  // void onImageMessage(const QByteArray& data, const QString& channel);
  void onKinectFrame(const QByteArray& data, const QString& channel);


protected:

  // CameraData* getCameraData(const QString& cameraName);
  // bool initCameraData(const QString& cameraName, CameraData* cameraData);

  vtkSmartPointer<vtkImageData> toVtkImage(CameraData* cameraData);

  BotParam* mBotParam;
  BotFrames* mBotFrames;

  ddLCMThread* mLCM;
  QMap<QString, QMap<int, QString> > mChannelMap;
  QMap<QString, bot_core::images_t> mImagesMessageMap;
  QMap<QString, ddLCMSubscriber*> mSubscribers;
  QMap<QString, CameraData*> mCameraData;

  KinectCalibration* kcal;
  int decimate_;
    
  uint8_t* rgb_buf_ ;

  vtkSmartPointer<vtkPolyData> mPolyData;
  QMutex mPolyDataMutex;

};

#endif
