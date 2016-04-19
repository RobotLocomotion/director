#ifndef __ddBotImageQueue_h
#define __ddBotImageQueue_h

#include <QObject>
#include <ddMacros.h>

#include "ddLCMThread.h"
#include "ddLCMSubscriber.h"
#include "ddAppConfigure.h"


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

class DD_APP_EXPORT ddBotImageQueue : public QObject
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

  ddBotImageQueue(QObject* parent=NULL);

  virtual ~ddBotImageQueue();

  // Adds an lcm subscriber for the given channel if one does not already exit.
  // Assumes the camera name is the same as the given channel name.  Camera name
  // is used to lookup camera parameters from botparam.  Not all image channels
  // have named cameras in botparam.
  bool addCameraStream(const QString& channel);

  // Adds an lcm subscriber for the given channel if one does not already exist.
  // Looks for camera parameters in botparam using the given cameraName.  Not
  // all image channels have named cameras in botparam.  The imageType, if >= 0,
  // is used to extract image_t message from images_t.
  bool addCameraStream(const QString& channel, const QString& cameraName, int imageType);

  void init(ddLCMThread* lcmThread, const QString& botConfigFile);

  qint64 getImage(const QString& cameraName, vtkImageData* image);
  qint64 getCurrentImageTime(const QString& cameraName);

  // Returns four xyz vectors as a 12 element list.  The vectors are rays
  // representing the top left, top right, bottom right, and bottom left
  // edges of the camera frustum.
  QList<double> getCameraFrustumBounds(const QString& cameraName);

  QList<double> unprojectPixel(const QString& cameraName, int px, int py);

  void colorizePoints(const QString& cameraName, vtkPolyData* polyData);

  void computeTextureCoords(const QString& cameraName, vtkPolyData* polyData);

  // Computes a point cloud with rgb and copies it into the given polyData.
  // The channel argument names an lcm channel where an images message are received
  // that contains disparity and color images.
  // decimation: power of 2 to reduce the data by (1,2,4,8...)
  // removeSize: remove disconnected components smaller than this size (in pixels), set=0 to skip
  void getPointCloudFromImages(const QString& channel, vtkPolyData* polyData, int decimation, int removeSize);

  // Project the points of the given polydata into image space.  The points must
  // already be in the camera coordinate system.  The points will be written to
  // in place.
  int projectPoints(const QString& cameraName, vtkPolyData* polyData);

  void getBodyToCameraTransform(const QString& cameraName, vtkTransform* transform);

  // Initializes the given transform to the camera projection matrix.
  // This will transform points from the camera coordinate system to image space.
  void getCameraProjectionTransform(const QString& cameraName, vtkTransform* transform);

  int getTransform(const QString& fromFrame, const QString& toFrame, qint64 utime, vtkTransform* transform);
  int getTransform(const QString& fromFrame, const QString& toFrame, vtkTransform* transform);

  QStringList getBotFrameNames() const;

protected slots:

  void onImagesMessage(const QByteArray& data, const QString& channel);
  void onImageMessage(const QByteArray& data, const QString& channel);

protected:

  CameraData* getCameraData(const QString& cameraName);
  bool initCameraData(const QString& cameraName, CameraData* cameraData);

  vtkSmartPointer<vtkImageData> toVtkImage(CameraData* cameraData);

  void colorizePoints(vtkPolyData* polyData, CameraData* cameraData);

  void computeTextureCoords(vtkPolyData* polyData, CameraData* cameraData);

  QList<double> getCameraFrustumBounds(CameraData* cameraData);

  int getTransform(std::string from_frame, std::string to_frame,
                     Eigen::Isometry3d& mat, qint64 utime);

  BotParam* mBotParam;
  BotFrames* mBotFrames;

  ddLCMThread* mLCM;
  QMap<QString, QMap<int, QString> > mChannelMap;
  QMap<QString, bot_core::images_t> mImagesMessageMap;
  QMap<QString, ddLCMSubscriber*> mSubscribers;
  QMap<QString, CameraData*> mCameraData;
};

#endif
