#include "ddBotImageQueue.h"


//-----------------------------------------------------------------------------
ddBotImageQueue::ddBotImageQueue(QObject* parent) : QObject(parent)
{
  mBotParam = 0;
  mBotFrames = 0;
}

//-----------------------------------------------------------------------------
ddBotImageQueue::~ddBotImageQueue()
{
  foreach (CameraData* cameraData, mCameraData.values())
  {
    delete cameraData;
  }
}

//-----------------------------------------------------------------------------
bool ddBotImageQueue::initCameraData(const QString& cameraName, CameraData* cameraData)
{
  cameraData->mName = cameraName.toAscii().data();

  cameraData->mCamTrans = bot_param_get_new_camtrans(mBotParam, cameraName.toAscii().data());
  if (!cameraData->mCamTrans)
  {
    printf("Failed to get BotCamTrans for camera: %s\n", qPrintable(cameraName));
    return false;
  }

  QString key = QString("cameras.") + cameraName + QString(".coord_frame");
  char* val = NULL;
  if (bot_param_get_str(mBotParam, key.toAscii().data(), &val) == 0)
  {
    cameraData->mCoordFrame = val;
    free(val);
  }
  else
  {
    printf("Failed to get coord_frame for camera: %s\n", qPrintable(cameraName));
    return false;
  }
  return true;

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

//-----------------------------------------------------------------------------
void ddBotImageQueue::init(ddLCMThread* lcmThread)
{
  bool useBotParamFromFile = true;

  if (useBotParamFromFile)
  {
    std::string configFile = std::string(getenv("DRC_BASE")) + "/software/config/drc_robot_05.cfg";
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

  mLCM = lcmThread;
  this->addCameraStream("CAMERACHEST_LEFT");
  this->addCameraStream("CAMERACHEST_RIGHT");
  this->addCameraStream("CAMERA_LEFT");
}

//-----------------------------------------------------------------------------
bool ddBotImageQueue::addCameraStream(const QString& cameraName)
{
  CameraData* cameraData = new CameraData;

  if (!this->initCameraData(cameraName, cameraData))
  {
    delete cameraData;
    return false;
  }


  ddLCMSubscriber* subscriber;
  QString channel = cameraName;


  bool useImagesMessage = false;

  if (useImagesMessage && cameraName == "CAMERA_LEFT")
  {
    channel = "CAMERA";
    subscriber = new ddLCMSubscriber(channel, this);
    this->connect(subscriber, SIGNAL(messageReceived(const QByteArray&, const QString&)), SLOT(onImagesMessage(const QByteArray&, const QString&)), Qt::DirectConnection);
  }
  else
  {
    subscriber = new ddLCMSubscriber(channel, this);
    this->connect(subscriber, SIGNAL(messageReceived(const QByteArray&, const QString&)), SLOT(onImageMessage(const QByteArray&, const QString&)), Qt::DirectConnection);
  }

  this->mChannelMap[channel] = cameraName;
  this->mSubscribers[cameraName] = subscriber;
  this->mCameraData[cameraName] = cameraData;
  mLCM->addSubscriber(subscriber);
  return true;
}

//-----------------------------------------------------------------------------
int ddBotImageQueue::getTransform(const QString& fromFrame, const QString& toFrame, quint64 utime, vtkTransform* transform)
{
  if (!transform)
    {
    return 0;
    }

  double matx[16];
  int status = bot_frames_get_trans_mat_4x4_with_utime(mBotFrames, fromFrame.toAscii().data(),  toFrame.toAscii().data(), utime, matx);

  vtkSmartPointer<vtkMatrix4x4> vtkmat = vtkSmartPointer<vtkMatrix4x4>::New();
  for (int i = 0; i < 4; ++i)
  {
    for (int j = 0; j < 4; ++j)
    {
      vtkmat->SetElement(i, j, matx[i*4+j]);
    }
  }

  transform->SetMatrix(vtkmat);
  return status;
}

//-----------------------------------------------------------------------------
int ddBotImageQueue::getTransform(std::string from_frame, std::string to_frame,
                   Eigen::Isometry3d & mat, vtkIdType utime)
{
  double matx[16];
  int status = bot_frames_get_trans_mat_4x4_with_utime( mBotFrames, from_frame.c_str(),  to_frame.c_str(), utime, matx);
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      mat(i,j) = matx[i*4+j];
    }
  }
  return status;
}

//-----------------------------------------------------------------------------
quint64 ddBotImageQueue::getImage(const QString& cameraName, vtkImageData* image)
{
  CameraData* cameraData = this->getCameraData(cameraName);
  if (!cameraData)
  {
    return 0;
  }

  image->DeepCopy(toVtkImage(cameraData));
  return cameraData->mImageMessage.utime;
}

//-----------------------------------------------------------------------------
quint64 ddBotImageQueue::getCurrentImageTime(const QString& cameraName)
{
  CameraData* cameraData = this->getCameraData(cameraName);
  if (!cameraData)
  {
    return 0;
  }

  return cameraData->mImageMessage.utime;
}

//-----------------------------------------------------------------------------
void ddBotImageQueue::colorizePoints(const QString& cameraName, vtkPolyData* polyData)
{
  CameraData* cameraData = this->getCameraData(cameraName);
  if (!cameraData)
  {
    return;
  }

  colorizePoints(polyData, cameraData);
}

//-----------------------------------------------------------------------------
void ddBotImageQueue::computeTextureCoords(const QString& cameraName, vtkPolyData* polyData)
{
  CameraData* cameraData = this->getCameraData(cameraName);
  if (!cameraData)
  {
    return;
  }

  this->computeTextureCoords(polyData, cameraData);
}

//-----------------------------------------------------------------------------
void ddBotImageQueue::getBodyToCameraTransform(const QString& cameraName, vtkTransform* transform)
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

  QMutexLocker locker(&cameraData->mMutex);
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

//-----------------------------------------------------------------------------
ddBotImageQueue::CameraData* ddBotImageQueue::getCameraData(const QString& cameraName)
{
  return this->mCameraData.value(cameraName, NULL);
}

//-----------------------------------------------------------------------------
void ddBotImageQueue::onImagesMessage(const QByteArray& data, const QString& channel)
{
  CameraData* cameraData = this->getCameraData(mChannelMap[channel]);

  multisense::images_t message;
  message.decode(data.data(), 0, data.size());

  QMutexLocker locker(&cameraData->mMutex);
  cameraData->mImageMessage = message.images[cameraData->mImageMessageIndex];
  cameraData->mImageBuffer.clear();
  this->getTransform("local", cameraData->mCoordFrame, cameraData->mLocalToCamera, cameraData->mImageMessage.utime);
  this->getTransform("utorso", cameraData->mCoordFrame, cameraData->mBodyToCamera, cameraData->mImageMessage.utime);
  //printf("got image %s: %d %d\n", cameraData->mName.c_str(), cameraData->mImageMessage.width, cameraData->mImageMessage.height);
}

//-----------------------------------------------------------------------------
void ddBotImageQueue::onImageMessage(const QByteArray& data, const QString& channel)
{
  CameraData* cameraData = this->getCameraData(mChannelMap[channel]);

  QMutexLocker locker(&cameraData->mMutex);
  cameraData->mImageMessage.decode(data.data(), 0, data.size());
  cameraData->mImageBuffer.clear();
  this->getTransform("local", cameraData->mCoordFrame, cameraData->mLocalToCamera, cameraData->mImageMessage.utime);
  this->getTransform("utorso", cameraData->mCoordFrame, cameraData->mBodyToCamera, cameraData->mImageMessage.utime);
  //printf("got image %s: %d %d\n", cameraData->mName.c_str(), cameraData->mImageMessage.width, cameraData->mImageMessage.height);
}

//-----------------------------------------------------------------------------
vtkSmartPointer<vtkImageData> ddBotImageQueue::toVtkImage(CameraData* cameraData)
{
  QMutexLocker locker(&cameraData->mMutex);

  size_t w = cameraData->mImageMessage.width;
  size_t h = cameraData->mImageMessage.height;
  size_t buf_size = w*h*3;

  if (buf_size == 0)
  {
    return vtkSmartPointer<vtkImageData>::New();
  }

  if (!cameraData->mImageBuffer.size())
  {
    if (cameraData->mImageMessage.pixelformat == bot_core::image_t::PIXEL_FORMAT_RGB)
    {
      cameraData->mImageBuffer = cameraData->mImageMessage.data;
    }
    else if (cameraData->mImageMessage.pixelformat != bot_core::image_t::PIXEL_FORMAT_MJPEG)
    {
      printf("Error: expected PIXEL_FORMAT_MJPEG for camera %s\n", cameraData->mName.c_str());
      return vtkSmartPointer<vtkImageData>::New();
    }
    else
    {

      //printf("jpeg decompress: %s\n", cameraData->mName.c_str());
      cameraData->mImageBuffer.resize(buf_size);
      jpeg_decompress_8u_rgb(cameraData->mImageMessage.data.data(), cameraData->mImageMessage.size, cameraData->mImageBuffer.data(), w, h, w*3);
    }
  }
  //else printf("already decompressed: %s\n", cameraData->mName.c_str());

  vtkSmartPointer<vtkImageData> image = vtkSmartPointer<vtkImageData>::New();

  image->SetWholeExtent(0, w-1, 0, h-1, 0, 0);
  image->SetSpacing(1.0, 1.0, 1.0);
  image->SetOrigin(0.0, 0.0, 0.0);
  image->SetExtent(image->GetWholeExtent());
  image->SetNumberOfScalarComponents(3);
  image->SetScalarType(VTK_UNSIGNED_CHAR);
  image->AllocateScalars();

  unsigned char* outPtr = static_cast<unsigned char*>(image->GetScalarPointer(0, 0, 0));

  std::copy(cameraData->mImageBuffer.begin(), cameraData->mImageBuffer.end(), outPtr);

  return image;
}

//-----------------------------------------------------------------------------
void ddBotImageQueue::colorizePoints(vtkPolyData* polyData, CameraData* cameraData)
{
  QMutexLocker locker(&cameraData->mMutex);

  size_t w = cameraData->mImageMessage.width;
  size_t h = cameraData->mImageMessage.height;
  size_t buf_size = w*h*3;

  bool computeDist = false;
  if (cameraData->mName == "CAMERACHEST_LEFT" || cameraData->mName == "CAMERACHEST_RIGHT")
  {
    computeDist = true;
  }

  if (!cameraData->mImageBuffer.size())
  {
    if (cameraData->mImageMessage.pixelformat != bot_core::image_t::PIXEL_FORMAT_MJPEG)
    {
      printf("Error: expected PIXEL_FORMAT_MJPEG for camera %s\n", cameraData->mName.c_str());
      return;
    }

    cameraData->mImageBuffer.resize(buf_size);
    jpeg_decompress_8u_rgb(cameraData->mImageMessage.data.data(), cameraData->mImageMessage.size, cameraData->mImageBuffer.data(), w, h, w*3);
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
    Eigen::Vector3d pt = cameraData->mLocalToCamera * ptLocal;

    double in[] = {pt[0], pt[1], pt[2]};
    double pix[3];
    if (bot_camtrans_project_point(cameraData->mCamTrans, in, pix) == 0)
    {
      int px = static_cast<int>(pix[0]);
      int py = static_cast<int>(pix[1]);

      if (px >= 0 && px < w && py >= 0 && py < h)
      {

        if (computeDist)
        {
          float u = pix[0] / (w-1);
          float v = pix[1] / (h-1);
          if  ( ((0.5 - u)*(0.5 - u) + (0.5 - v)*(0.5 -v)) > 0.2 )
          {
            continue;
          }
        }

        size_t bufIndex = w*py*3 + px*3;
        rgb->SetComponent(i, 0, cameraData->mImageBuffer[bufIndex + 0]);
        rgb->SetComponent(i, 1, cameraData->mImageBuffer[bufIndex + 1]);
        rgb->SetComponent(i, 2, cameraData->mImageBuffer[bufIndex + 2]);
      }
    }
  }
}

//-----------------------------------------------------------------------------
void ddBotImageQueue::computeTextureCoords(vtkPolyData* polyData, CameraData* cameraData)
{
  QMutexLocker locker(&cameraData->mMutex);

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
    //Eigen::Vector3d pt = cameraData->mLocalToCamera * ptLocal;
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
