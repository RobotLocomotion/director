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

  ddBotImageQueue(QObject* parent=NULL) : QObject(parent),
    mMultisenseSubscriber("CAMERA"),
    mChestLeftSubscriber("CAMERACHEST_LEFT"),
    mChestRightSubscriber("CAMERACHEST_RIGHT")
  {

    mBotParam = 0;
    mBotFrames = 0;

    this->connect(&mMultisenseSubscriber, SIGNAL(messageReceived(const QByteArray&)), SLOT(onMultisenseImages(const QByteArray&)));
    this->connect(&mChestLeftSubscriber, SIGNAL(messageReceived(const QByteArray&)), SLOT(onChestLeft(const QByteArray&)));
    this->connect(&mChestRightSubscriber, SIGNAL(messageReceived(const QByteArray&)), SLOT(onChestRight(const QByteArray&)));
  }

  virtual ~ddBotImageQueue()
  {
  }


  struct CameraData
  {
    std::string mName;
    std::string mCoordFrame;
    BotCamTrans* mCamTrans;
    bot_core::image_t mImageMessage;
    Eigen::Isometry3d mLocalToCamera;
    Eigen::Isometry3d mBodyToCamera;
    std::vector<uint8_t> mImageBuffer;

    CameraData()
    {
      mImageMessage.width = 0;
      mImageMessage.height = 0;
    }

    ~CameraData()
    {
      if (mCamTrans) bot_camtrans_destroy(mCamTrans);
    }
  };


  void initCamera(std::string name, CameraData& cameraData)
  {
    cameraData.mName = name;

    cameraData.mCamTrans = bot_param_get_new_camtrans(mBotParam, name.c_str());
    if (!cameraData.mCamTrans)
    {
      printf("Failed to get BotCamTrans for camera: %s\n", name.c_str());
    }

    std::string key("cameras.");
    key += (name + ".coord_frame");
    char* val = NULL;
    if (bot_param_get_str(mBotParam, key.c_str(), &val) == 0)
    {
      cameraData.mCoordFrame = val;
      free(val);
    }
    else
    {
      printf("Failed to get coord_frame for camera: %s\n", name.c_str());
    }


    /*
    double K00 = bot_camtrans_get_focal_length_x(sub->mCamTrans);
    double K11 = bot_camtrans_get_focal_length_y(sub->mCamTrans);
    double K01 = bot_camtrans_get_skew(sub->mCamTrans);
    double K02 = bot_camtrans_get_principal_x(sub->mCamTrans);
    double K12 = bot_camtrans_get_principal_y(sub->mCamTrans);
    sub->mProjectionMatrix = Eigen::Matrix4f::Zero();
    sub->mProjectionMatrix(0,0) = K00;
    sub->mProjectionMatrix(0,1) = K01;
    sub->mProjectionMatrix(0,2) = K02;
    sub->mProjectionMatrix(1,1) = K11;
    sub->mProjectionMatrix(1,2) = K12;
    sub->mProjectionMatrix(2,3) = 1;
    sub->mProjectionMatrix(3,2) = 1;
    sub->mImageWidth = bot_camtrans_get_width(sub->mCamTrans);
    sub->mImageHeight = bot_camtrans_get_width(sub->mCamTrans);
    */
  }

  void init(ddLCMThread* lcmThread)
  {

    bool useBotParamFromFile = true;

    if (useBotParamFromFile)
    {
      std::string configFile = std::string(getenv("DRC_BASE")) + "/software/config/drc_robot_02.cfg";
      mBotParam = bot_param_new_from_file(configFile.c_str());
    }
    else
    {
      while (!mBotParam)
        mBotParam = bot_param_new_from_server(lcmThread->lcmHandle()->getUnderlyingLCM(), 0);
    }

    mBotFrames = bot_frames_get_global(lcmThread->lcmHandle()->getUnderlyingLCM(), mBotParam);


    //char** cameraNames =  bot_param_get_all_camera_names(mBotParam);
    //for (int i = 0; cameraNames[i] != 0; ++i)
    //{
    //  printf("camera: %s\n", cameraNames[i]);
    //}


    this->initCamera("CAMERACHEST_LEFT", mChestLeft);
    this->initCamera("CAMERACHEST_RIGHT", mChestRight);
    this->initCamera("CAMERA_LEFT", mHeadLeft);

    lcmThread->addSubscriber(&mMultisenseSubscriber);
    lcmThread->addSubscriber(&mChestLeftSubscriber);
    lcmThread->addSubscriber(&mChestRightSubscriber);
  }


  int getTransform(std::string from_frame, std::string to_frame,
                     Eigen::Isometry3d & mat, vtkIdType utime)
  {
    int status;
    double matx[16];
    status = bot_frames_get_trans_mat_4x4_with_utime( mBotFrames, from_frame.c_str(),  to_frame.c_str(), utime, matx);
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        mat(i,j) = matx[i*4+j];
      }
    }
    return status;
  }

  void getImage(const QString& name, vtkImageData* image)
  {
    if (name == "CAMERA_LEFT")
    {
      image->DeepCopy(toVtkImage(mHeadLeft));
    }
    else if (name == "CAMERACHEST_LEFT")
    {
      image->DeepCopy(toVtkImage(mChestLeft));
    }
    else if (name == "CAMERACHEST_RIGHT")
    {
      image->DeepCopy(toVtkImage(mChestRight));
    }
  }

  void colorizeLidar(vtkPolyData* polyData)
  {
    colorizeLidar(polyData, mChestLeft);
    colorizeLidar(polyData, mHeadLeft);
  }

  void computeTextureCoords(vtkPolyData* polyData)
  {
    computeTextureCoords(polyData, &mChestLeft);
    computeTextureCoords(polyData, &mChestRight);
    computeTextureCoords(polyData, &mHeadLeft);
  }

  void getBodyToCameraTransform(const QString& cameraName, vtkTransform* transform)
  {
    if (!transform)
    {
      return;
    }

    transform->Identity();

    CameraData* cameraData = this->getCameraData(cameraName);
    if (!cameraData)
    {
      return;
    }

    Eigen::Isometry3d mat = cameraData->mBodyToCamera;
    vtkSmartPointer<vtkMatrix4x4> vtkmat = vtkSmartPointer<vtkMatrix4x4>::New();
    for (int i = 0; i < 4; ++i)
    {
      for (int j = 0; j < 4; ++j)
      {
        vtkmat->SetElement(i, j, mat(i,j));
      }
    }
    transform->SetMatrix(vtkmat);
  }


  void colorizeLidar(vtkPolyData* polyData, CameraData& cameraData)
  {
    size_t w = cameraData.mImageMessage.width;
    size_t h = cameraData.mImageMessage.height;
    size_t buf_size = w*h*3;

    if (!cameraData.mImageBuffer.size())
    {
      if (cameraData.mImageMessage.pixelformat != bot_core::image_t::PIXEL_FORMAT_MJPEG)
      {
        printf("Error: expected PIXEL_FORMAT_MJPEG for camera %s\n", cameraData.mName.c_str());
        return;
      }

      cameraData.mImageBuffer.resize(buf_size);
      jpeg_decompress_8u_rgb(cameraData.mImageMessage.data.data(), cameraData.mImageMessage.size, cameraData.mImageBuffer.data(), w, h, w*3);
    }


    vtkSmartPointer<vtkUnsignedCharArray> rgb = vtkUnsignedCharArray::SafeDownCast(polyData->GetPointData()->GetArray("rgb"));
    if (!rgb)
    {
      rgb = vtkSmartPointer<vtkUnsignedCharArray>::New();
      rgb->SetName("rgb");
      rgb->SetNumberOfComponents(3);
      rgb->SetNumberOfTuples(polyData->GetNumberOfPoints());
      polyData->GetPointData()->AddArray(rgb);

      rgb->FillComponent(0, 255);
      rgb->FillComponent(1, 255);
      rgb->FillComponent(2, 255);
    }

    const vtkIdType nPoints = polyData->GetNumberOfPoints();
    for (vtkIdType i = 0; i < nPoints; ++i)
    {
      Eigen::Vector3d ptLocal;
      polyData->GetPoint(i, ptLocal.data());
      Eigen::Vector3d pt = cameraData.mLocalToCamera * ptLocal;

      double in[] = {pt[0], pt[1], pt[2]};
      double pix[3];
      if (bot_camtrans_project_point(cameraData.mCamTrans, in, pix) == 0)
      {
        int px = static_cast<int>(pix[0]);
        int py = static_cast<int>(pix[1]);

        if (px >= 0 && px < w && py >= 0 && py < h)
        {
          size_t bufIndex = w*py*3 + px*3;
          rgb->SetComponent(i, 0, cameraData.mImageBuffer[bufIndex + 0]);
          rgb->SetComponent(i, 1, cameraData.mImageBuffer[bufIndex + 1]);
          rgb->SetComponent(i, 2, cameraData.mImageBuffer[bufIndex + 2]);
        }
      }
    }
  }


protected slots:

  void onMultisenseImages(const QByteArray& data)
  {
    multisense::images_t message;
    message.decode(data.data(), 0, data.size());

    mHeadLeft.mImageMessage = message.images[0];
    mHeadLeft.mImageBuffer.clear();
    this->getTransform("local", mHeadLeft.mCoordFrame, mHeadLeft.mLocalToCamera, mHeadLeft.mImageMessage.utime);
    this->getTransform("utorso", mHeadLeft.mCoordFrame, mHeadLeft.mBodyToCamera, mHeadLeft.mImageMessage.utime);
    //printf("got image %s: %d %d\n", mHeadLeft.mName.c_str(), mHeadLeft.mImageMessage.width, mHeadLeft.mImageMessage.height);
  }

  void onChestLeft(const QByteArray& data)
  {
    mChestLeft.mImageMessage.decode(data.data(), 0, data.size());
    mChestLeft.mImageBuffer.clear();
    this->getTransform("local", mChestLeft.mCoordFrame, mChestLeft.mLocalToCamera, mChestLeft.mImageMessage.utime);
    this->getTransform("utorso", mChestLeft.mCoordFrame, mChestLeft.mBodyToCamera, mChestLeft.mImageMessage.utime);
    //printf("got image %s: %d %d\n", mChestLeft.mName.c_str(), mChestLeft.mImageMessage.width, mChestLeft.mImageMessage.height);
  }

  void onChestRight(const QByteArray& data)
  {
    mChestRight.mImageMessage.decode(data.data(), 0, data.size());
    mChestRight.mImageBuffer.clear();
    this->getTransform("local", mChestRight.mCoordFrame, mChestRight.mLocalToCamera, mChestRight.mImageMessage.utime);
    this->getTransform("utorso", mChestRight.mCoordFrame, mChestRight.mBodyToCamera, mChestRight.mImageMessage.utime);
    //printf("got image %s: %d %d\n", mChestRight.mName.c_str(), mChestRight.mImageMessage.width, mChestRight.mImageMessage.height);
  }

  vtkSmartPointer<vtkImageData> toVtkImage(CameraData& cameraData)
  {

    size_t w = cameraData.mImageMessage.width;
    size_t h = cameraData.mImageMessage.height;
    size_t buf_size = w*h*3;

    if (buf_size == 0)
    {
      return vtkSmartPointer<vtkImageData>::New();
    }

    if (!cameraData.mImageBuffer.size())
    {
      if (cameraData.mImageMessage.pixelformat != bot_core::image_t::PIXEL_FORMAT_MJPEG)
      {
        printf("Error: expected PIXEL_FORMAT_MJPEG for camera %s\n", cameraData.mName.c_str());
        return vtkSmartPointer<vtkImageData>::New();
      }

      cameraData.mImageBuffer.resize(buf_size);
      jpeg_decompress_8u_rgb(cameraData.mImageMessage.data.data(), cameraData.mImageMessage.size, cameraData.mImageBuffer.data(), w, h, w*3);
    }

    vtkSmartPointer<vtkImageData> image = vtkSmartPointer<vtkImageData>::New();

    image->SetWholeExtent(0, w-1, 0, h-1, 0, 0);
    image->SetSpacing(1.0, 1.0, 1.0);
    image->SetOrigin(0.0, 0.0, 0.0);
    image->SetExtent(image->GetWholeExtent());
    image->SetNumberOfScalarComponents(3);
    image->SetScalarType(VTK_UNSIGNED_CHAR);
    image->AllocateScalars();

    unsigned char* outPtr = static_cast<unsigned char*>(image->GetScalarPointer(0, 0, 0));

    std::copy(cameraData.mImageBuffer.begin(), cameraData.mImageBuffer.end(), outPtr);

    return image;
  }

protected:

  CameraData* getCameraData(const QString& cameraName)
  {
    if (cameraName == "CAMERA_LEFT")
    {
      return &mHeadLeft;
    }
    else if (cameraName == "CAMERACHEST_LEFT")
    {
      return &mChestLeft;
    }
    else if (cameraName == "CAMERACHEST_RIGHT")
    {
      return &mChestRight;
    }
    return 0;
  }

  void computeTextureCoords(vtkPolyData* polyData, CameraData* cameraData)
  {
    size_t w = cameraData->mImageMessage.width;
    size_t h = cameraData->mImageMessage.height;

    bool computeDist = false;
    if (cameraData->mName == "CAMERACHEST_LEFT" || cameraData->mName == "CAMERACHEST_RIGHT")
    {
      computeDist = true;
    }

    std::string arrayName = "tcoords_" + cameraData->mName;
    vtkSmartPointer<vtkFloatArray> tcoords = vtkFloatArray::SafeDownCast(polyData->GetPointData()->GetArray(arrayName.c_str()));
    if (!tcoords)
    {
      tcoords = vtkSmartPointer<vtkFloatArray>::New();
      tcoords->SetName(arrayName.c_str());
      tcoords->SetNumberOfComponents(2);
      tcoords->SetNumberOfTuples(polyData->GetNumberOfPoints());
      polyData->GetPointData()->AddArray(tcoords);

      tcoords->FillComponent(0, -1);
      tcoords->FillComponent(1, -1);
    }

    const vtkIdType nPoints = polyData->GetNumberOfPoints();
    for (vtkIdType i = 0; i < nPoints; ++i)
    {
      Eigen::Vector3d ptLocal;
      polyData->GetPoint(i, ptLocal.data());
      //Eigen::Vector3d pt = cameraData.mLocalToCamera * ptLocal;
      //Eigen::Vector3d pt = cameraData->mBodyToCamera * ptLocal;
      Eigen::Vector3d pt = ptLocal;

      double in[] = {pt[0], pt[1], pt[2]};
      double pix[3];
      if (bot_camtrans_project_point(cameraData->mCamTrans, in, pix) == 0)
      {
        float u = pix[0] / (w-1);
        float v = pix[1] / (h-1);

        //if (u >= 0 && u <= 1.0 && v >= 0 && v <= 1.0)
        //{
          //if (computeDist &&  ((0.5 - u)*(0.5 - u) + (0.5 - v)*(0.5 -v)) > 0.14)
          //{
          //  continue;
          //}
          tcoords->SetComponent(i, 0, u);
          tcoords->SetComponent(i, 1, v);
        //}
      }
    }
  }

  BotParam* mBotParam;
  BotFrames* mBotFrames;

  ddLCMSubscriber mMultisenseSubscriber;
  ddLCMSubscriber mChestLeftSubscriber;
  ddLCMSubscriber mChestRightSubscriber;


  CameraData mChestLeft;
  CameraData mChestRight;
  CameraData mHeadLeft;


};

#endif
