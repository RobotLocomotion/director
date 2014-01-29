#ifndef __ddBotImageQueue_h
#define __ddBotImageQueue_h

#include <QObject>
#include <ddMacros.h>

#include "ddLCMThread.h"
#include "ddLCMSubscriber.h"

#include <string>
#include <sstream>

#include <Eigen/Geometry>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core/image_t.hpp>
#include <lcmtypes/multisense/images_t.hpp>

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

class ddBotImageQueue : public QObject
 {
  Q_OBJECT

public:

  class CameraData
  {
    public:

    int mImageMessageIndex;
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
      mImageMessageIndex = 0;
      mImageMessage.width = 0;
      mImageMessage.height = 0;
      mImageMessage.utime = 0;
    }

    ~CameraData()
    {
      if (mCamTrans) bot_camtrans_destroy(mCamTrans);
    }
  };

  ddBotImageQueue(QObject* parent=NULL);

  virtual ~ddBotImageQueue();

  bool addCameraStream(const QString& cameraName);

  void init(ddLCMThread* lcmThread);

  quint64 getImage(const QString& cameraName, vtkImageData* image);
  quint64 getCurrentImageTime(const QString& cameraName);

  void colorizePoints(const QString& cameraName, vtkPolyData* polyData);

  void computeTextureCoords(const QString& cameraName, vtkPolyData* polyData);

  void getBodyToCameraTransform(const QString& cameraName, vtkTransform* transform);

  int getTransform(const QString& fromFrame, const QString& toFrame, quint64 utime, vtkTransform* transform);

protected slots:

  void onImagesMessage(const QByteArray& data, const QString& channel);
  void onImageMessage(const QByteArray& data, const QString& channel);

protected:

  CameraData* getCameraData(const QString& cameraName);
  bool initCameraData(const QString& cameraName, CameraData* cameraData);

  vtkSmartPointer<vtkImageData> toVtkImage(CameraData* cameraData);

  void colorizePoints(vtkPolyData* polyData, CameraData* cameraData);

  void computeTextureCoords(vtkPolyData* polyData, CameraData* cameraData);

  int getTransform(std::string from_frame, std::string to_frame,
                     Eigen::Isometry3d & mat, vtkIdType utime);

  BotParam* mBotParam;
  BotFrames* mBotFrames;

  ddLCMThread* mLCM;
  QMap<QString, QString> mChannelMap;
  QMap<QString, ddLCMSubscriber*> mSubscribers;
  QMap<QString, CameraData*> mCameraData;
};

#endif
