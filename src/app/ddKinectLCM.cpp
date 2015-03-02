#include <boost/thread.hpp>

#include "ddKinectLCM.h"

#include <zlib.h>

#include <vtkIdTypeArray.h>
#include <vtkCellArray.h>
#include <vtkNew.h>

#include <multisense_utils/multisense_utils.hpp>
#include <lcmtypes/kinect/frame_msg_t.hpp>
#include <lcmtypes/kinect_frame_msg_t.h>


//-----------------------------------------------------------------------------
ddKinectLCM::ddKinectLCM(QObject* parent) : QObject(parent)
{
  mBotParam = 0;
  mBotFrames = 0;
}

//-----------------------------------------------------------------------------
ddKinectLCM::~ddKinectLCM()
{
  foreach (CameraData* cameraData, mCameraData.values())
  {
    delete cameraData;
  }
}

//-----------------------------------------------------------------------------
bool ddKinectLCM::initCameraData(const QString& cameraName, CameraData* cameraData)
{
  cameraData->mName = cameraName.toAscii().data();
  cameraData->mHasCalibration = true;

  cameraData->mCamTrans = bot_param_get_new_camtrans(mBotParam, cameraName.toAscii().data());
  if (!cameraData->mCamTrans)
  {
    printf("Failed to get BotCamTrans for camera: %s\n", qPrintable(cameraName));
    cameraData->mHasCalibration = false;
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
    cameraData->mHasCalibration = false;
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
void ddKinectLCM::init(ddLCMThread* lcmThread, const QString& botConfigFile)
{
  
  mLCM = lcmThread;
  
  QString channelName = "KINECT_FRAME";
  ddLCMSubscriber* subscriber = new ddLCMSubscriber(channelName, this);

  this->connect(subscriber, SIGNAL(messageReceived(const QByteArray&, const QString&)),
          SLOT(onKinectFrame(const QByteArray&, const QString&)), Qt::DirectConnection);

  mLCM->addSubscriber(subscriber);


  kcal = kinect_calib_new();
  kcal->intrinsics_depth.fx = 576.09757860;
  kcal->intrinsics_depth.cx = 321.06398107;
  kcal->intrinsics_depth.cy = 242.97676897;
  kcal->intrinsics_rgb.fx = 576.09757860;
  kcal->intrinsics_rgb.cx = 321.06398107;
  kcal->intrinsics_rgb.cy = 242.97676897;
  kcal->intrinsics_rgb.k1 = 0; // none given so far
  kcal->intrinsics_rgb.k2 = 0; // none given so far
  kcal->shift_offset = 1079.4753;
  kcal->projector_depth_baseline = 0.07214;
  //double rotation[9];
  double rotation[]={0.999999, -0.000796, 0.001256, 0.000739, 0.998970, 0.045368, -0.001291, -0.045367, 0.998970};
  double depth_to_rgb_translation[] ={ -0.015756, -0.000923, 0.002316};
  memcpy(kcal->depth_to_rgb_rot, rotation, 9*sizeof(double)); 
  memcpy(kcal->depth_to_rgb_translation, depth_to_rgb_translation  , 3*sizeof(double));  


  // Data buffer
  rgb_buf_ = (uint8_t*) malloc(10*1024 * 1024 * sizeof(uint8_t)); 


  decimate_ =1.0;



}






#include <zlib.h>


// Copied from kinect-lcm
static inline void
_matrix_vector_multiply_3x4_4d (const double m[12], const double v[4],
    double result[3])
{
  result[0] = m[0]*v[0] + m[1]*v[1] + m[2] *v[2] + m[3] *v[3];
  result[1] = m[4]*v[0] + m[5]*v[1] + m[6] *v[2] + m[7] *v[3];
  result[2] = m[8]*v[0] + m[9]*v[1] + m[10]*v[2] + m[11]*v[3];
}



void unpack_kinect_frame(const kinect_frame_msg_t *msg, uint8_t* rgb_data, KinectCalibration* kcal, int decimate_, 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud){

  /////////////////////////////////////////////////////////////////////
  /// 1.1 RGB:
  // TODO check width, height
  if(msg->image.image_data_format == KINECT_IMAGE_MSG_T_VIDEO_RGB) {
    memcpy(rgb_data, msg->image.image_data,
        msg->depth.width * msg->depth.height * 3);
  } else if(msg->image.image_data_format == KINECT_IMAGE_MSG_T_VIDEO_RGB_JPEG) {
    jpeg_decompress_8u_rgb (msg->image.image_data, msg->image.image_data_nbytes,
        rgb_data, msg->image.width, msg->image.height, msg->image.width* 3);
    //jpegijg_decompress_8u_rgb(msg->image.image_data, msg->image.image_data_nbytes,
    //        rgb_data, msg->image.width, msg->image.height, msg->image.width* 3);
  }

  /////////////////////////////////////////////////////////////////////
  /// 1.2. DEPTH:
  uint8_t* uncompress_buffer = NULL;
  int uncompress_buffer_size = 0;
  const uint8_t* depth_data =NULL; //= msg->depth.depth_data;
  // 1.2.1 De-compress if necessary:
  if(msg->depth.compression != KINECT_DEPTH_MSG_T_COMPRESSION_NONE) {
    //std:: cout << "compression \n ";
    if(msg->depth.uncompressed_size > uncompress_buffer_size) {
      uncompress_buffer_size = msg->depth.uncompressed_size;
      uncompress_buffer = (uint8_t*) realloc(uncompress_buffer, uncompress_buffer_size);
    }
    unsigned long dlen = msg->depth.uncompressed_size;
    int status = uncompress(uncompress_buffer, &dlen, 
        msg->depth.depth_data, msg->depth.depth_data_nbytes);
    if(status != Z_OK) {
      return;
    }
    depth_data =(uint8_t*) uncompress_buffer;
  }else{
    depth_data = (uint8_t*) msg->depth.depth_data;
  }

  int npixels = msg->depth.width * msg->depth.height;
  if (msg->depth.depth_data_format == KINECT_DEPTH_MSG_T_DEPTH_11BIT){ 
    /////////////////////////////////////////////////////////////////////
    // Freenect Data
    uint16_t* disparity_array = (uint16_t*) malloc(msg->depth.width * msg->depth.height * sizeof(uint16_t)); 
    if(G_BYTE_ORDER == G_LITTLE_ENDIAN) {
      int16_t* rdd = (int16_t*) depth_data;
      int i;
      for(i=0; i<npixels; i++) {
        int d = rdd[i];
        disparity_array[i] = d;
      }
    } else {
      fprintf(stderr, "Big endian systems not supported\n");
    }

    /// 2 Calculate transformation matrices:
    double depth_to_rgb_uvd[12];
    double depth_to_depth_xyz[16];
    kinect_calib_get_depth_uvd_to_rgb_uvw_3x4(kcal, depth_to_rgb_uvd);
    kinect_calib_get_depth_uvd_to_depth_xyz_4x4(kcal, depth_to_depth_xyz);
    double depth_to_depth_xyz_trans[16];
    //_matrix_transpose_4x4d(depth_to_depth_xyz, depth_to_depth_xyz_trans);
    bot_matrix_transpose_4x4d(depth_to_depth_xyz, depth_to_depth_xyz_trans);

    // 3 for each depth point find the corresponding xyz and then RGB
    //   then put into PCL structure
    cloud->width    =(int) (msg->depth.width/ (double) decimate_) ;
    cloud->height   =(int) (msg->depth.height/ (double) decimate_);
    cloud->is_dense = false;
    cloud->points.resize (cloud->width * cloud->height);
    double xyzw2[4];
    int j2=0;
    // NB: the order of these loop was changed... aug 2011. important
    for(int v=0; v<msg->depth.height; v=v+ decimate_) { // t2b state->height 480
      for(int u=0; u<msg->depth.width; u=u+decimate_ ) {  //l2r state->width 640
        // 3.4.1 compute distorted pixel coordinates
        uint16_t disparity = disparity_array[v*msg->depth.width+u];
        double uvd_depth[4] = { u, v, disparity, 1 };
        double uvd_rgb[3];
        _matrix_vector_multiply_3x4_4d(depth_to_rgb_uvd, uvd_depth, uvd_rgb);
        double uv_rect[2] = {
            uvd_rgb[0] / uvd_rgb[2],
            uvd_rgb[1] / uvd_rgb[2]
        };
        double uv_dist[2];
        kinect_calib_distort_rgb_uv(kcal, uv_rect, uv_dist);
        int u_rgb = uv_dist[0] + 0.5;
        int v_rgb = uv_dist[1] + 0.5;
        uint8_t r, g, b;
        if(u_rgb >= msg->depth.width || u_rgb < 0 || v_rgb >= msg->depth.height || v_rgb < 0) {
          r = g = b = 0;
        } else {
          r = rgb_data[v_rgb*msg->depth.width*3 + u_rgb*3 + 0];
          g = rgb_data[v_rgb*msg->depth.width*3 + u_rgb*3 + 1];
          b = rgb_data[v_rgb*msg->depth.width*3 + u_rgb*3 + 2];
        }
        // 3.4.2 find the xyz location of the points:
        bot_matrix_multiply(depth_to_depth_xyz, 4, 4,
            uvd_depth, 4, 1, xyzw2);

        cloud->points[j2].y = -xyzw2[0]/xyzw2[3];//y right+ (check)
        cloud->points[j2].z = -xyzw2[1]/xyzw2[3];//z up+
        cloud->points[j2].x = xyzw2[2]/xyzw2[3]; //x forward+
        // was bgr...
        cloud->points[j2].b =b;
        cloud->points[j2].r =r;
        cloud->points[j2].g =g;
        j2++;
      }
    }
    free(disparity_array);

  }else if(msg->depth.depth_data_format == KINECT_DEPTH_MSG_T_DEPTH_MM  ){ 
    /////////////////////////////////////////////////////////////////////
    // Openni Data
    // 1.2.2 unpack raw byte data into float values in mm

    // NB: no depth return is given 0 range - and becomes 0,0,0 here
    const uint16_t* val = reinterpret_cast<const uint16_t*>( depth_data );

    cloud->width    =(int) (msg->depth.width/ (double) decimate_) ;
    cloud->height   =(int) (msg->depth.height/ (double) decimate_); 
    cloud->is_dense = false;
    cloud->points.resize (cloud->width * cloud->height);
    int j2=0;
    for(int v=0; v<msg->depth.height; v=v+ decimate_) { // t2b self->height 480
      for(int u=0; u<msg->depth.width; u=u+decimate_ ) {  //l2r self->width 640
        uint8_t r = rgb_data[v*msg->depth.width*3 + u*3 + 0];
        uint8_t g = rgb_data[v*msg->depth.width*3 + u*3 + 1];
        uint8_t b = rgb_data[v*msg->depth.width*3 + u*3 + 2];
        double constant = 1.0f / kcal->intrinsics_rgb.fx ;
        double disparity_d = val[v*msg->depth.width+u]  / 1000.0; // convert to m
        cloud->points[j2].y =  - (((double) u)- 319.50)*disparity_d*constant; //y right+ (check)
        cloud->points[j2].z = - (((double) v)- 239.50)*disparity_d*constant;  //z up+
        cloud->points[j2].x = disparity_d;  //x forward+
        if (disparity_d==0){ // place null points at negative range... arbitarty decision
          double disparity_d = -0.1; // convert to m
          cloud->points[j2].y =  - (((double) u)- 319.50)*disparity_d*constant; //y right+ (check)
          cloud->points[j2].z =  -(((double) v)- 239.50)*disparity_d*constant;  //z up+
          cloud->points[j2].x = disparity_d;  //x forward+
        }

        cloud->points[j2].b =b;
        cloud->points[j2].r =r;
        cloud->points[j2].g =g;
        j2++;
      }
    }         
  }

  if(msg->depth.compression != KINECT_DEPTH_MSG_T_COMPRESSION_NONE) {
    free(uncompress_buffer); // memory leak bug fixed
  }
}








void ddKinectLCM::onKinectFrame(const QByteArray& data, const QString& channel)
{
  
  kinect_frame_msg_t message;
  //message.decode(data.data(), 0, data.size());
  kinect_frame_msg_t_decode (data.data(), 0, 1e9, &message);

  printf("got kinect frame %d\n", message.timestamp);

  //convert to pcl object


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  unpack_kinect_frame(&message, rgb_buf_, kcal, decimate_,  cloud);
  //polyData->ShallowCopy(PolyDataFromPointCloud(cloud));
  printf("Decoded point cloud with %u\n", cloud->size());
  

}




//-----------------------------------------------------------------------------
bool ddKinectLCM::addCameraStream(const QString& channel)
{
  return this->addCameraStream(channel, channel, -1);
}

//-----------------------------------------------------------------------------
bool ddKinectLCM::addCameraStream(const QString& channel, const QString& cameraName, int imageType)
{
  if (!this->mCameraData.contains(cameraName))
  {
    CameraData* cameraData = new CameraData;
    if (!this->initCameraData(cameraName, cameraData))
    {
      delete cameraData;
      return false;
    }

    this->mCameraData[cameraName] = cameraData;
  }

  this->mChannelMap[channel][imageType] = cameraName;

  if (!this->mSubscribers.contains(channel))
  {
    ddLCMSubscriber* subscriber = new ddLCMSubscriber(channel, this);

    if (imageType >= 0)
    {
      this->connect(subscriber, SIGNAL(messageReceived(const QByteArray&, const QString&)),
          SLOT(onImagesMessage(const QByteArray&, const QString&)), Qt::DirectConnection);
    }
    else
    {
      this->connect(subscriber, SIGNAL(messageReceived(const QByteArray&, const QString&)),
          SLOT(onImageMessage(const QByteArray&, const QString&)), Qt::DirectConnection);
    }

    this->mSubscribers[channel] = subscriber;
    mLCM->addSubscriber(subscriber);
  }


  return true;
}

//-----------------------------------------------------------------------------
QStringList ddKinectLCM::getBotFrameNames() const
{
  int nFrames = bot_frames_get_num_frames(mBotFrames);
  char** namesArray = bot_frames_get_frame_names(mBotFrames);

  QStringList names;
  for (int i = 0; i < nFrames; ++i)
  {
    names << namesArray[i];
  }

  return names;
}

//-----------------------------------------------------------------------------
int ddKinectLCM::getTransform(const QString& fromFrame, const QString& toFrame, qint64 utime, vtkTransform* transform)
{
  if (!transform)
    {
    return 0;
    }

  double matx[16];
  int status = bot_frames_get_trans_mat_4x4_with_utime(mBotFrames, fromFrame.toAscii().data(),  toFrame.toAscii().data(), utime, matx);
  if (!status)
    {
    return 0;
    }

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
int ddKinectLCM::getTransform(const QString& fromFrame, const QString& toFrame, vtkTransform* transform)
{
  if (!transform)
    {
    return 0;
    }

  double matx[16];
  int status = bot_frames_get_trans_mat_4x4(mBotFrames, fromFrame.toAscii().data(),  toFrame.toAscii().data(), matx);
  if (!status)
    {
    return 0;
    }

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
int ddKinectLCM::getTransform(std::string from_frame, std::string to_frame,
                   Eigen::Isometry3d & mat, qint64 utime)
{
  double matx[16];
  int status = bot_frames_get_trans_mat_4x4_with_utime( mBotFrames, from_frame.c_str(),  to_frame.c_str(), utime, matx);
  if (!status)
    {
    return 0;
    }

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      mat(i,j) = matx[i*4+j];
    }
  }
  return status;
}

//-----------------------------------------------------------------------------
qint64 ddKinectLCM::getImage(const QString& cameraName, vtkImageData* image)
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
qint64 ddKinectLCM::getCurrentImageTime(const QString& cameraName)
{
  CameraData* cameraData = this->getCameraData(cameraName);
  if (!cameraData)
  {
    return 0;
  }

  return cameraData->mImageMessage.utime;
}

//-----------------------------------------------------------------------------
void ddKinectLCM::colorizePoints(const QString& cameraName, vtkPolyData* polyData)
{
  CameraData* cameraData = this->getCameraData(cameraName);
  if (!cameraData)
  {
    return;
  }

  colorizePoints(polyData, cameraData);
}

//-----------------------------------------------------------------------------
void ddKinectLCM::computeTextureCoords(const QString& cameraName, vtkPolyData* polyData)
{
  CameraData* cameraData = this->getCameraData(cameraName);
  if (!cameraData)
  {
    return;
  }

  this->computeTextureCoords(polyData, cameraData);
}

//-----------------------------------------------------------------------------
QList<double> ddKinectLCM::getCameraFrustumBounds(const QString& cameraName)
{
  CameraData* cameraData = this->getCameraData(cameraName);
  if (!cameraData)
  {
    return QList<double>();
  }

  return this->getCameraFrustumBounds(cameraData);
}

//-----------------------------------------------------------------------------
void ddKinectLCM::getBodyToCameraTransform(const QString& cameraName, vtkTransform* transform)
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
ddKinectLCM::CameraData* ddKinectLCM::getCameraData(const QString& cameraName)
{
  return this->mCameraData.value(cameraName, NULL);
}

//-----------------------------------------------------------------------------
void ddKinectLCM::onImagesMessage(const QByteArray& data, const QString& channel)
{
  bot_core::images_t& message = this->mImagesMessageMap[channel];
  message.decode(data.data(), 0, data.size());

  const QMap<int, QString> cameraNameMap = mChannelMap[channel];

  for (QMap<int, QString>::const_iterator itr = cameraNameMap.constBegin(); itr != cameraNameMap.end(); ++itr)
  {
    int imageType = itr.key();
    const QString& cameraName = itr.value();

    bot_core::image_t* imageMessage = 0;
    for (int i = 0; i < message.n_images; ++i)
    {
      if (message.image_types[i] == imageType)
      {
        imageMessage = &message.images[i];
        break;
      }
    }

    if (!imageMessage)
    {
      return;
    }


    CameraData* cameraData = this->getCameraData(cameraName);

    QMutexLocker locker(&cameraData->mMutex);
    cameraData->mImageMessage = *imageMessage;
    cameraData->mImageBuffer.clear();

    if (cameraData->mHasCalibration)
    {
      this->getTransform("local", cameraData->mCoordFrame, cameraData->mLocalToCamera, cameraData->mImageMessage.utime);
      this->getTransform("utorso", cameraData->mCoordFrame, cameraData->mBodyToCamera, cameraData->mImageMessage.utime);
    }

    //printf("got image %s: %d %d\n", cameraData->mName.c_str(), cameraData->mImageMessage.width, cameraData->mImageMessage.height);
  }
}

//-----------------------------------------------------------------------------
void ddKinectLCM::onImageMessage(const QByteArray& data, const QString& channel)
{
  const QString& cameraName = mChannelMap[channel][-1];

  CameraData* cameraData = this->getCameraData(cameraName);


  QMutexLocker locker(&cameraData->mMutex);
  cameraData->mImageMessage.decode(data.data(), 0, data.size());
  cameraData->mImageBuffer.clear();

  if (cameraData->mHasCalibration)
  {
    this->getTransform("local", cameraData->mCoordFrame, cameraData->mLocalToCamera, cameraData->mImageMessage.utime);
    this->getTransform("utorso", cameraData->mCoordFrame, cameraData->mBodyToCamera, cameraData->mImageMessage.utime);
  }

  //printf("got image %s: %d %d\n", cameraData->mName.c_str(), cameraData->mImageMessage.width, cameraData->mImageMessage.height);
}

//-----------------------------------------------------------------------------
vtkSmartPointer<vtkImageData> ddKinectLCM::toVtkImage(CameraData* cameraData)
{
  QMutexLocker locker(&cameraData->mMutex);

  size_t w = cameraData->mImageMessage.width;
  size_t h = cameraData->mImageMessage.height;
  size_t buf_size = w*h*3;

  if (buf_size == 0)
  {
    return vtkSmartPointer<vtkImageData>::New();
  }

  int nComponents = 3;
  int componentType = VTK_UNSIGNED_CHAR;

  if (!cameraData->mImageBuffer.size())
  {
    if (cameraData->mImageMessage.pixelformat == bot_core::image_t::PIXEL_FORMAT_RGB)
    {
      cameraData->mImageBuffer = cameraData->mImageMessage.data;
    }
    /*
    else if (cameraData->mImageMessage.pixelformat == bot_core::image_t::PIXEL_FORMAT_GRAY)
    {
      cameraData->mImageBuffer.resize(w*h*2);
      unsigned long len = cameraData->mImageBuffer.size();

      uncompress(cameraData->mImageBuffer.data(), &len, cameraData->mImageMessage.data.data(), cameraData->mImageMessage.size);

      componentType = VTK_UNSIGNED_SHORT;
      nComponents = 1;
    }
    */
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
  image->SetNumberOfScalarComponents(nComponents);
  image->SetScalarType(componentType);
  image->AllocateScalars();

  unsigned char* outPtr = static_cast<unsigned char*>(image->GetScalarPointer(0, 0, 0));

  std::copy(cameraData->mImageBuffer.begin(), cameraData->mImageBuffer.end(), outPtr);

  return image;
}

namespace {

//----------------------------------------------------------------------------
vtkSmartPointer<vtkCellArray> NewVertexCells(vtkIdType numberOfVerts)
{
  vtkNew<vtkIdTypeArray> cells;
  cells->SetNumberOfValues(numberOfVerts*2);
  vtkIdType* ids = cells->GetPointer(0);
  for (vtkIdType i = 0; i < numberOfVerts; ++i)
    {
    ids[i*2] = 1;
    ids[i*2+1] = i;
    }

  vtkSmartPointer<vtkCellArray> cellArray = vtkSmartPointer<vtkCellArray>::New();
  cellArray->SetCells(numberOfVerts, cells.GetPointer());
  return cellArray;
}

//----------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData> PolyDataFromPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  vtkIdType nr_points = cloud->points.size();

  vtkNew<vtkPoints> points;
  points->SetDataTypeToFloat();
  points->SetNumberOfPoints(nr_points);

  vtkNew<vtkUnsignedCharArray> rgbArray;
  rgbArray->SetName("rgb_colors");
  rgbArray->SetNumberOfComponents(3);
  rgbArray->SetNumberOfTuples(nr_points);

  vtkIdType j = 0;    // true point index
  for (vtkIdType i = 0; i < nr_points; ++i)
  {
    // Check if the point is invalid
    if (!pcl_isfinite (cloud->points[i].x) ||
        !pcl_isfinite (cloud->points[i].y) ||
        !pcl_isfinite (cloud->points[i].z))
      continue;

    float point[3] = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
    unsigned char color[3] = {cloud->points[i].r, cloud->points[i].g, cloud->points[i].b};
    points->SetPoint(j, point);
    rgbArray->SetTupleValue(j, color);
    j++;
  }
  nr_points = j;
  points->SetNumberOfPoints(nr_points);
  rgbArray->SetNumberOfTuples(nr_points);

  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  polyData->SetPoints(points.GetPointer());
  polyData->GetPointData()->AddArray(rgbArray.GetPointer());
  polyData->SetVerts(NewVertexCells(nr_points));
  return polyData;
}


};

//-----------------------------------------------------------------------------
void ddKinectLCM::getPointCloudFromImages(const QString& channel, vtkPolyData* polyData, int decimation, int removeSize)
{
  if (!this->mImagesMessageMap.contains(channel))
  {
    printf("no images received on channel: %s\n", qPrintable(channel));
    return;
  }

  bot_core::images_t& msg = this->mImagesMessageMap[channel];

  // Read the camera calibration from params (including baseline:
  CameraData* cameraData = this->getCameraData("CAMERA_LEFT");
  QString key = QString("coordinate_frames.CAMERA_RIGHT.initial_transform.translation");
  double baseline = 0.07; // an approximate value
  if (!bot_param_get_double(mBotParam, key.toAscii().data(), &baseline) == 0){
    printf("CAMERA_RIGHT baseline not found\n");
    return;
  }
  cv::Mat_<double> Q_(4, 4, 0.0);
  Q_(0,0) = Q_(1,1) = 1.0;
  Q_(3,2) = 1.0 / baseline;
  Q_(0,3) = -bot_camtrans_get_principal_x( cameraData->mCamTrans ); // cx
  Q_(1,3) = -bot_camtrans_get_principal_y( cameraData->mCamTrans ); // cy
  Q_(2,3) = bot_camtrans_get_focal_length_x( cameraData->mCamTrans ); // fx
  Q_(3,3) = 0;//(stereo_params_.right.cx - stereo_params_.left.cx ) / baseline;

  static multisense_utils m;
  m.set_decimate(decimation);
  m.set_remove_size( removeSize );

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  m.unpack_multisense(&msg, Q_, cloud);
  polyData->ShallowCopy(PolyDataFromPointCloud(cloud));
}

//-----------------------------------------------------------------------------
int ddKinectLCM::projectPoints(const QString& cameraName, vtkPolyData* polyData)
{

  CameraData* cameraData = this->getCameraData(cameraName);

  if (!cameraData->mHasCalibration)
  {
    printf("Error: computeTextureCoords, no calibration data for: %s\n", cameraData->mName.c_str());
    return -1;
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
      polyData->GetPoints()->SetPoint(i, pix);
    }
  }

  return 1;
}

//-----------------------------------------------------------------------------
void ddKinectLCM::colorizePoints(vtkPolyData* polyData, CameraData* cameraData)
{
  if (!cameraData->mHasCalibration)
  {
    printf("Error: colorizePoints, no calibration data for: %s\n", cameraData->mName.c_str());
    return;
  }

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
QList<double> ddKinectLCM::unprojectPixel(const QString& cameraName, int px, int py)
{
  double xyz[3];
  CameraData* cameraData = this->getCameraData(cameraName);
  if (!cameraData)
  {
    return QList<double>();
  }

  bot_camtrans_unproject_pixel(cameraData->mCamTrans, px, py, xyz);
  QList<double> result;
  result << xyz[0] << xyz[1] << xyz[2];
  return result;
}

//-----------------------------------------------------------------------------
QList<double> ddKinectLCM::getCameraFrustumBounds(CameraData* cameraData)
{
  double width = bot_camtrans_get_image_width(cameraData->mCamTrans);
  double height = bot_camtrans_get_image_width(cameraData->mCamTrans);
  double ray[4][3];

  bot_camtrans_unproject_pixel(cameraData->mCamTrans, 0, 0, &ray[0][0]);
  bot_camtrans_unproject_pixel(cameraData->mCamTrans, width, 0, &ray[1][0]);
  bot_camtrans_unproject_pixel(cameraData->mCamTrans, width, height, &ray[2][0]);
  bot_camtrans_unproject_pixel(cameraData->mCamTrans, 0, height, &ray[3][0]);

  QList<double> rays;
  for (int i = 0; i < 4; ++i)
  {
    Eigen::Vector3d pt(&ray[i][0]);
    rays.append(pt[0]);
    rays.append(pt[1]);
    rays.append(pt[2]);
  }

  return rays;
}

//-----------------------------------------------------------------------------
void ddKinectLCM::computeTextureCoords(vtkPolyData* polyData, CameraData* cameraData)
{
  if (!cameraData->mHasCalibration)
  {
    printf("Error: computeTextureCoords, no calibration data for: %s\n", cameraData->mName.c_str());
    return;
  }

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
