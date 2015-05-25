#include <boost/thread.hpp>

#include "ddKinectLCM.h"

#include <zlib.h>

#include <vtkIdTypeArray.h>
#include <vtkCellArray.h>
#include <vtkNew.h>

#include <multisense_utils/multisense_utils.hpp>
#include <lcmtypes/kinect/frame_msg_t.hpp>
#include <lcmtypes/kinect_frame_msg_t.h>
#include <zlib.h>


//-----------------------------------------------------------------------------
ddKinectLCM::ddKinectLCM(QObject* parent) : QObject(parent)
{
  mPolyData = vtkSmartPointer<vtkPolyData>::New();
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

  /*
  Originally was this:
  kcal = kinect_calib_new();
  kcal->intrinsics_depth.fx = 528.01442863461716;//was 576.09757860;
  kcal->intrinsics_depth.cx = 321.06398107;
  kcal->intrinsics_depth.cy = 242.97676897;
  kcal->intrinsics_rgb.fx = 528.01442863461716;//576.09757860; ... 528 seems to be better, emperically, march 2015
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
  */


  // This is in full agreement with Kintinuous: (calibrationAsus.yml)
  // NB: if changing this, it should be kept in sync
  kcal = kinect_calib_new();
  kcal->intrinsics_depth.fx = 528.01442863461716;//was 576.09757860;
  kcal->intrinsics_depth.cx = 320;
  kcal->intrinsics_depth.cy = 267.0;
  kcal->intrinsics_rgb.fx = 528.01442863461716;//576.09757860; ... 528 seems to be better, emperically, march 2015
  kcal->intrinsics_rgb.cx = 320;
  kcal->intrinsics_rgb.cy = 267.0;
  kcal->intrinsics_rgb.k1 = 0; // none given so far
  kcal->intrinsics_rgb.k2 = 0; // none given so far
  kcal->shift_offset = 1090.0;
  kcal->projector_depth_baseline = 0.075;
  //double rotation[9];
  double rotation[]={0.999999, -0.000796, 0.001256, 0.000739, 0.998970, 0.045368, -0.001291, -0.045367, 0.998970};
  double depth_to_rgb_translation[] ={ -0.015756, -0.000923, 0.002316};
  memcpy(kcal->depth_to_rgb_rot, rotation, 9*sizeof(double));
  memcpy(kcal->depth_to_rgb_translation, depth_to_rgb_translation  , 3*sizeof(double));



  // Data buffer
  rgb_buf_ = (uint8_t*) malloc(10*1024 * 1024 * sizeof(uint8_t)); 


  decimate_ =1.0;
}



//-----------------------------------------------------------------------------
static inline void
_matrix_vector_multiply_3x4_4d (const double m[12], const double v[4],
    double result[3])
{
  result[0] = m[0]*v[0] + m[1]*v[1] + m[2] *v[2] + m[3] *v[3];
  result[1] = m[4]*v[0] + m[5]*v[1] + m[6] *v[2] + m[7] *v[3];
  result[2] = m[8]*v[0] + m[9]*v[1] + m[10]*v[2] + m[11]*v[3];
}


//-----------------------------------------------------------------------------
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

        cloud->points[j2].x = xyzw2[0]/xyzw2[3]; //x right+
        cloud->points[j2].y = xyzw2[1]/xyzw2[3]; //y down+
        cloud->points[j2].z = xyzw2[2]/xyzw2[3]; //z forward+

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

        if (disparity_d!=0){
          cloud->points[j2].x = (((double) u)- kcal->intrinsics_depth.cx)*disparity_d*constant; //x right+
          cloud->points[j2].y = (((double) v)- kcal->intrinsics_depth.cy)*disparity_d*constant; //y down+
          cloud->points[j2].z = disparity_d;  //z forward+
          cloud->points[j2].b =b;
          cloud->points[j2].r =r;
          cloud->points[j2].g =g;
          j2++;
        }

      }
    }
    cloud->points.resize (j2);
  }

  if(msg->depth.compression != KINECT_DEPTH_MSG_T_COMPRESSION_NONE) {
    free(uncompress_buffer); 
  }
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

  vtkIdType j = 0;    
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
void ddKinectLCM::onKinectFrame(const QByteArray& data, const QString& channel)
{
  
  kinect_frame_msg_t message;

  kinect_frame_msg_t_decode (data.data(), 0, 1e9, &message);

  //printf("got kinect frame %d\n", message.timestamp);

  //convert to pcl object:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  unpack_kinect_frame(&message, rgb_buf_, kcal, decimate_,  cloud);

  kinect_frame_msg_t_decode_cleanup(&message);
  //printf("Decoded point cloud with %u\n", cloud->size());

  vtkSmartPointer<vtkPolyData> polyData = PolyDataFromPointCloud(cloud);

  QMutexLocker locker(&this->mPolyDataMutex);
  this->mPolyData = polyData;
  this->mUtime = message.timestamp;
}


//-----------------------------------------------------------------------------
qint64 ddKinectLCM::getPointCloudFromKinect(vtkPolyData* polyDataRender)
{
  QMutexLocker locker(&this->mPolyDataMutex);
  polyDataRender->ShallowCopy(this->mPolyData);
  return this->mUtime;
}

