/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkMapServerSource.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkMapServerSource.h"


#include "vtkPolyData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkSmartPointer.h"
#include "vtkTransform.h"
#include "vtkTransformPolyDataFilter.h"
#include "vtkNew.h"


#include <vtkIdList.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkImageData.h>

#include <vtkMultisenseUtils.h>

#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>

#include <lcm/lcm-cpp.hpp>

//#include <lcmtypes/drc_lcmtypes.hpp>
#include <lcmtypes/drc/map_image_t.hpp>
#include <lcmtypes/drc/map_cloud_t.hpp>
#include <lcmtypes/drc/map_octree_t.hpp>
#include <lcmtypes/drc/map_scans_t.hpp>
#include <lcmtypes/drc/data_request_t.hpp>


#include <maps/LcmTranslator.hpp>
#include <maps/DepthImageView.hpp>
#include <maps/DepthImage.hpp>
#include <maps/PointCloudView.hpp>
#include <maps/OctreeView.hpp>
#include <maps/ScanBundleView.hpp>


#include <sys/select.h>
#include <deque>

namespace
{

const int WORKSPACE_DEPTH_VIEW_ID = drc::data_request_t::DEPTH_MAP_WORKSPACE;

/* commented out due to conflict with vtkMultisenseUtils.h version
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
*/

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


  if (cloud->is_dense)
  {
    for (vtkIdType i = 0; i < nr_points; ++i) {
      float point[3] = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
      unsigned char color[3] = {cloud->points[i].r, cloud->points[i].g, cloud->points[i].b}; 
      points->SetPoint(i, point);
      rgbArray->SetTupleValue(i, color);
    }
  }
  else
  {
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
  }

  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  polyData->SetPoints(points.GetPointer());
  polyData->GetPointData()->AddArray(rgbArray.GetPointer());
  polyData->SetVerts(NewVertexCells(nr_points));
  return polyData;
}

//----------------------------------------------------------------------------
void AddZCoordinateArray(vtkPolyData* polyData)
{
  const vtkIdType nPoints = polyData->GetNumberOfPoints();

  vtkSmartPointer<vtkDoubleArray> zcoord = vtkSmartPointer<vtkDoubleArray>::New();
  zcoord->SetName("z");
  zcoord->SetNumberOfComponents(1);
  zcoord->SetNumberOfTuples(nPoints);

  for (vtkIdType i = 0; i < nPoints; ++i)
  {
    double z = polyData->GetPoint(i)[2];
    if (!pcl_isfinite(z))
    {
      z = 0.0;
    }
    zcoord->SetValue(i, z);
  }

  polyData->GetPointData()->AddArray(zcoord);
}

//----------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData> ConvertMesh(maps::TriangleMesh::Ptr mesh)
{
  const size_t nPoints = mesh->mVertices.size();
  const size_t nTris = mesh->mFaces.size();
  const bool hasNormals = (mesh->mNormals.size() != 0);

  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  points->SetDataTypeToDouble();
  points->SetNumberOfPoints(nPoints);

  vtkSmartPointer<vtkDoubleArray> normalsArray = vtkSmartPointer<vtkDoubleArray>::New();
  normalsArray->SetName("Normals");
  normalsArray->SetNumberOfComponents(3);
  normalsArray->SetNumberOfTuples(nPoints);

  for (size_t i = 0; i < nPoints; ++i)
  {
    points->SetPoint(i, mesh->mVertices[i].data());
  }

  if (hasNormals)
  {
    for (size_t i = 0; i < nPoints; ++i)
    {
      normalsArray->SetTuple(i, mesh->mNormals[i].data());
    }
  }


  vtkSmartPointer<vtkCellArray> cellArray = vtkSmartPointer<vtkCellArray>::New();

  vtkIdType ids[3];
  for (vtkIdType i = 0; i < nTris; ++i)
    {
    ids[0] = mesh->mFaces[i][0];
    ids[1] = mesh->mFaces[i][1];
    ids[2] = mesh->mFaces[i][2];
    cellArray->InsertNextCell(3, ids);
    }

  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  polyData->SetPoints(points);
  polyData->SetPolys(cellArray);

  if (hasNormals)
  {
    polyData->GetPointData()->SetNormals(normalsArray);
  }

  return polyData;
}

//----------------------------------------------------------------------------
class MapData
{
public:
  uint64_t Id;
  vtkSmartPointer<vtkPolyData> Data;
  vtkSmartPointer<vtkPolyData> Mesh;
  vtkSmartPointer<vtkImageData> DepthImage;
  vtkSmartPointer<vtkTransform> Transform;
};

//----------------------------------------------------------------------------
class LCMListener
{
public:

  LCMListener()
  {
    this->ShouldStop = true;
    this->NewData = false;
    this->MaxNumberOfDatasets = 20;
    this->ViewIds = vtkSmartPointer<vtkIntArray>::New();
    this->LastScanBundleUtime = 0;
    this->CurrentScanBundleId = 0;
    this->DistanceRange[0] = 0.25;
    this->DistanceRange[1] = 4.0;
    this->EdgeAngleThreshold = 30;  // degrees

    this->LCMHandle = boost::shared_ptr<lcm::LCM>(new lcm::LCM);
    if(!this->LCMHandle->good())
    {
      std::cerr <<"ERROR: lcm is not good()" <<std::endl;
    }

    this->LCMHandle->subscribe( "MAP_DEPTH", &LCMListener::depthHandler, this);
    this->LCMHandle->subscribe( "MAP_DEBUG", &LCMListener::depthHandler, this);
    this->LCMHandle->subscribe( "MAP_CLOUD", &LCMListener::cloudHandler, this);
    this->LCMHandle->subscribe( "MAP_OCTREE", &LCMListener::octreeHandler, this);
    this->LCMHandle->subscribe( "MAP_SCANS", &LCMListener::scanBundleHandler, this);
  }


  void cloudHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const drc::map_cloud_t* msg)
  {
    this->HandleNewData(msg);
  }

  void depthHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const drc::map_image_t* msg)
  {
    this->HandleNewData(msg);
  }

  void octreeHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const drc::map_octree_t* msg)
  {
    this->HandleNewData(msg);
  }

  void scanBundleHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const drc::map_scans_t* msg)
  {
    this->HandleNewData(msg);
  }

  void SetDistanceRange(double distanceRange[2])
  {
    this->DistanceRange[0] = distanceRange[0];
    this->DistanceRange[1] = distanceRange[1];
  }

  void SetEdgeAngleThreshold(double edgeAngleThreshold)
  {
    this->EdgeAngleThreshold = edgeAngleThreshold;
  }

  double GetEdgeAngleThreshold()
  {
    return this->EdgeAngleThreshold;
  }

  bool CheckForNewData()
  {
    boost::lock_guard<boost::mutex> lock(this->Mutex);
    bool newData = this->NewData;
    this->NewData = false;
    return newData;
  }

  bool WaitForLCM(double timeout)
  {
    int lcmFd = this->LCMHandle->getFileno();

    timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = timeout * 1e6;

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(lcmFd, &fds);

    int status = select(lcmFd + 1, &fds, 0, 0, &tv);
    if (status == -1 && errno != EINTR)
    {
      printf("select() returned error: %d\n", errno);
    }
    else if (status == -1 && errno == EINTR)
    {
      printf("select() interrupted\n");
    }
    return (status > 0 && FD_ISSET(lcmFd, &fds));
  }

  void ThreadLoopWithSelect()
  {
    while (!this->ShouldStop)
    {
      const double timeoutInSeconds = 0.3;
      bool lcmReady = this->WaitForLCM(timeoutInSeconds);

      if (this->ShouldStop)
      {
        break;
      }

      if (lcmReady)
      {
        if (this->LCMHandle->handle() != 0)
        {
          printf("lcm->handle() returned non-zero\n");
          break;
        }
      }
    }

  }


  void ThreadLoop()
  {
    while (!this->ShouldStop)
    {
      if (this->LCMHandle->handle() != 0)
      {
        printf("lcm->handle() returned non-zero\n");
        break;
      }
    }
  }

  void Start()
  {
    if (this->Thread)
      {
      return;
      }

    this->ShouldStop = false;
    this->Thread = boost::shared_ptr<boost::thread>(
      new boost::thread(boost::bind(&LCMListener::ThreadLoopWithSelect, this)));
  }

  void Stop()
  {
    if (this->Thread)
      {
      this->ShouldStop = true;
      this->Thread->interrupt();
      this->Thread->join();
      this->Thread.reset();
      }
  }

  std::vector<vtkSmartPointer<vtkPolyData> > GetDatasets(int viewId)
  {
    std::vector<vtkSmartPointer<vtkPolyData> > polyData;

    boost::lock_guard<boost::mutex> lock(this->Mutex);
    std::deque<MapData>& datasets = this->Datasets[viewId];
    for (size_t i = 0; i < datasets.size(); ++i)
      {
      polyData.push_back(datasets[i].Data);
      }

    return polyData;
  }

  std::vector<double> GetTimesteps(int viewId)
  {
    boost::lock_guard<boost::mutex> lock(this->Mutex);

    std::vector<double> timesteps;
    std::deque<MapData>& datasets = this->Datasets[viewId];
    for (int i = 0; i < datasets.size(); ++i)
      {
      timesteps.push_back(i);
      }
    return timesteps;
  }

  vtkSmartPointer<vtkPolyData> GetDatasetForTime(int viewId, int timestep)
  {
    boost::lock_guard<boost::mutex> lock(this->Mutex);
    std::deque<MapData>& datasets = this->Datasets[viewId];
    if (timestep >= 0 && timestep < datasets.size())
      {
      return datasets[timestep].Data;
      }
    else
      {
      return 0;
      }
  }

  vtkIdType GetCurrentMapId(int viewId)
  {
    boost::lock_guard<boost::mutex> lock(this->Mutex);
    std::map<int, vtkIdType>::const_iterator itr = this->CurrentMapIds.find(viewId);
    if (itr == this->CurrentMapIds.end())
      {
        return -1;
      }
    return itr->second;
  }

  void GetDataForMapId(int viewId, vtkIdType mapId, vtkPolyData* polyData)
  {
    if (!polyData)
      {
      return;
      }

    boost::lock_guard<boost::mutex> lock(this->Mutex);
    std::deque<MapData>& datasets = this->Datasets[viewId];
    for (size_t i = 0; i < datasets.size(); ++i)
      {
      if (datasets[i].Id == mapId)
        {
        polyData->DeepCopy(datasets[i].Data);
        }
      }
  }

  void GetMeshForMapId(int viewId, vtkIdType mapId, vtkPolyData* polyData)
  {
    if (!polyData)
      {
      return;
      }

    boost::lock_guard<boost::mutex> lock(this->Mutex);
    std::deque<MapData>& datasets = this->Datasets[viewId];
    for (size_t i = 0; i < datasets.size(); ++i)
      {
      if (datasets[i].Id == mapId)
        {
        polyData->DeepCopy(datasets[i].Mesh);
        }
      }
  }

  void GetDataForMapId(int viewId, vtkIdType mapId, vtkImageData* imageData, vtkTransform* transform)
  {
    if (!imageData || !transform)
      {
      return;
      }

    boost::lock_guard<boost::mutex> lock(this->Mutex);
    std::deque<MapData>& datasets = this->Datasets[viewId];
    for (size_t i = 0; i < datasets.size(); ++i)
      {
      if (datasets[i].Id == mapId)
        {
        imageData->DeepCopy(datasets[i].DepthImage);
        transform->DeepCopy(datasets[i].Transform);
        }
      }
  }
  uint64_t GetLastScanBundleUTime()
  {
    return this->LastScanBundleUtime;
  }

  vtkIntArray* GetViewIds()
  {
    this->ViewIds->SetNumberOfTuples(this->CurrentMapIds.size());

    std::map<int, vtkIdType>::const_iterator itr;
    int i;
    for (i = 0, itr = this->CurrentMapIds.begin(); itr != this->CurrentMapIds.end(); ++itr, ++i)
      {
      this->ViewIds->SetValue(i, itr->first);
      }

    return this->ViewIds;
  }

protected:

  void UpdateDequeSize(std::deque<MapData>& datasets)
  {
    if (this->MaxNumberOfDatasets <= 0)
      {
      return;
      }
    while (datasets.size() >= this->MaxNumberOfDatasets)
      {
      datasets.pop_front();
      }
  }

  vtkIdType GetNextMapId(int viewId)
  {
    std::map<int, vtkIdType>::iterator itr = this->CurrentMapIds.find(viewId);
    if (itr == this->CurrentMapIds.end())
      {
        this->CurrentMapIds[viewId] = -1;
        itr = this->CurrentMapIds.find(viewId);
      }

    return ++itr->second;
  }


  vtkSmartPointer<vtkTransform> ToVtkTransform(const Eigen::Projective3f& mat)
  {
    vtkSmartPointer<vtkMatrix4x4> vtkmat = vtkSmartPointer<vtkMatrix4x4>::New();
    for (int i = 0; i < 4; ++i)
      {
      for (int j = 0; j < 4; ++j)
        {
        vtkmat->SetElement(i, j, mat(i,j));
        }
      }

    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    transform->SetMatrix(vtkmat);
    return transform;
  }

  vtkSmartPointer<vtkImageData> ConvertDepthImage(std::shared_ptr<maps::DepthImage> depthImage)
  {
    int width = depthImage->getWidth();
    int height = depthImage->getHeight();

    vtkSmartPointer<vtkImageData> image = vtkSmartPointer<vtkImageData>::New();
    image->SetWholeExtent(0, width-1, 0, height-1, 0, 0);
    image->SetSpacing(1.0, 1.0, 1.0);
    image->SetOrigin(0.0, 0.0, 0.0);
    image->SetExtent(image->GetWholeExtent());
    image->SetNumberOfScalarComponents(1);
    image->SetScalarType(VTK_FLOAT);
    image->AllocateScalars();

    std::vector<float> imageData = depthImage->getData(maps::DepthImage::TypeDepth);

    /*
    float invalidValue = depthImage->getInvalidValue(maps::DepthImage::TypeDepth);
    float replacementValue = 0.0
    for (int i = 0; i < imageData.size(); ++i)
      {
      if (imageData[i] == invalidValue)
        {
          imageData[i] = replacementValue;
        }
      }
    */

    float* outPtr = static_cast<float*>(image->GetScalarPointer(0, 0, 0));
    std::copy(imageData.begin(), imageData.end(), outPtr);

    return image;
  }

  void HandleNewData(const drc::map_image_t* msg)
  {
    int viewId = msg->view_id;
    maps::DepthImageView depthImageView;
    maps::LcmTranslator::fromLcm(*msg, depthImageView);

    maps::PointCloud::Ptr pointCloud = depthImageView.getAsPointCloud();
    maps::TriangleMesh::Ptr mesh = depthImageView.getAsMesh();

    MapData mapData;
    mapData.Data = PolyDataFromPointCloud(pointCloud);
    mapData.DepthImage = this->ConvertDepthImage(depthImageView.getDepthImage());
    mapData.Mesh = ConvertMesh(mesh);
    AddZCoordinateArray(mapData.Mesh);
    mapData.Transform = this->ToVtkTransform(depthImageView.getTransform());

    //printf("storing depth map %d.  %d points.  (view id %d)\n", mapData.Id, mapData.Data->GetNumberOfPoints(), viewId);

    // store data
    boost::lock_guard<boost::mutex> lock(this->Mutex);
    std::deque<MapData>& datasets = this->Datasets[viewId];
    mapData.Id = this->GetNextMapId(viewId);
    datasets.push_back(mapData);
    this->UpdateDequeSize(datasets);
    this->NewData = true;
  }

  void HandleNewData(const drc::map_cloud_t* msg)
  {
    int viewId = msg->view_id;
    maps::PointCloudView cloudView;
    maps::LcmTranslator::fromLcm(*msg, cloudView);

    maps::PointCloud::Ptr pointCloud = cloudView.getAsPointCloud();

    MapData mapData;
    mapData.Data = PolyDataFromPointCloud(pointCloud);

    //printf("storing cloud map %d.  %d points.  (view id %d)\n", mapData.Id, mapData.Data->GetNumberOfPoints(), viewId);

    // store data    
    boost::lock_guard<boost::mutex> lock(this->Mutex);
    std::deque<MapData>& datasets = this->Datasets[viewId];
    mapData.Id = this->GetNextMapId(viewId);
    datasets.push_back(mapData);
    this->UpdateDequeSize(datasets);
    this->NewData = true;
  }

  void HandleNewData(const drc::map_octree_t* msg)
  {
    int viewId = msg->view_id;
    maps::OctreeView octreeView;
    maps::LcmTranslator::fromLcm(*msg, octreeView);

    maps::PointCloud::Ptr pointCloud = octreeView.getAsPointCloud();

    MapData mapData;
    mapData.Data = PolyDataFromPointCloud(pointCloud);
    mapData.Transform = this->ToVtkTransform(octreeView.getTransform());
    mapData.Mesh = mapData.Data;

    // store data
    boost::lock_guard<boost::mutex> lock(this->Mutex);
    std::deque<MapData>& datasets = this->Datasets[viewId];
    mapData.Id = this->GetNextMapId(viewId);
    datasets.push_back(mapData);
    this->UpdateDequeSize(datasets);
    this->NewData = true;
  }

  void HandleNewData(const drc::map_scans_t* msg)
  {
    if (msg->utime == this->LastScanBundleUtime) return;
    this->LastScanBundleUtime = msg->utime;
    this->CurrentScanBundleId++;

    if (msg->num_scans == 0) return;

    int viewId = msg->view_id;
    maps::ScanBundleView bundleView;
    maps::LcmTranslator::fromLcm(*msg, bundleView);

    // convert/copy scans
    auto scans = bundleView.getScans();
    std::vector<ScanLineData> scanLines(scans.size());
    for (int i = 0; i < (int)scans.size(); ++i) {
      const auto& in = scans[i];
      auto& out = scanLines[i];
      out.Revolution = this->CurrentScanBundleId;
      out.ScanLineId = i;
      out.SpindleAngle = (float)i/(scans.size()-1)*180; // TODO: this is approximate at best
      out.ScanToLocalStart = in->getStartPose().cast<double>();
      out.ScanToLocalEnd = in->getEndPose().cast<double>();

      bot_core::planar_lidar_t msg;
      msg.utime = in->getTimestamp();
      msg.rad0 = in->getThetaMin();
      msg.radstep = in->getThetaStep();
      msg.ranges = in->getRanges();
      msg.nranges = msg.ranges.size();
      msg.intensities = in->getIntensities();
      if (msg.intensities.size() == 0) {
        msg.intensities.resize(msg.nranges);
        std::fill(msg.intensities.begin(), msg.intensities.end(), 0);
      }
      msg.nintensities = msg.nranges;
      out.msg = msg;
    }

    MapData mapData;
    mapData.Data = GetPointCloudFromScanLines(scanLines, this->DistanceRange, this->EdgeAngleThreshold);
    mapData.Transform = this->ToVtkTransform(bundleView.getTransform());
    mapData.Mesh = mapData.Data;

    // store data
    boost::lock_guard<boost::mutex> lock(this->Mutex);
    std::deque<MapData>& datasets = this->Datasets[viewId];
    mapData.Id = this->GetNextMapId(viewId);
    datasets.push_back(mapData);
    this->UpdateDequeSize(datasets);
    this->NewData = true;
  }

  bool NewData;
  bool ShouldStop;
  int MaxNumberOfDatasets;
  vtkIdType CurrentMapId;
  vtkSmartPointer<vtkIntArray> ViewIds;

  double EdgeAngleThreshold;
  double DistanceRange[2];

  boost::mutex Mutex;
  int64_t LastScanBundleUtime;
  int32_t CurrentScanBundleId;

  std::map<int, std::deque<MapData> > Datasets;
  std::map<int, vtkIdType> CurrentMapIds;

  boost::shared_ptr<lcm::LCM> LCMHandle;

  boost::shared_ptr<boost::thread> Thread;

};


} // end namespace

//----------------------------------------------------------------------------
class vtkMapServerSource::vtkInternal
{
public:

  vtkInternal()
  {
    this->Listener = boost::shared_ptr<LCMListener>(new LCMListener);
  }

  ~vtkInternal()
  {
  }

  boost::shared_ptr<LCMListener> Listener;
};

//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkMapServerSource);

//----------------------------------------------------------------------------
vtkMapServerSource::vtkMapServerSource()
{
  this->Internal = new vtkInternal;
  this->SetNumberOfInputPorts(0);
  this->SetNumberOfOutputPorts(1);
  this->DistanceRange[0] = 0.25;
  this->DistanceRange[1] = 4.0;
  this->Internal->Listener->SetDistanceRange(this->DistanceRange);
}

//----------------------------------------------------------------------------
vtkMapServerSource::~vtkMapServerSource()
{
  this->Stop();
  delete this->Internal;
}

//----------------------------------------------------------------------------
void vtkMapServerSource::Start()
{
  this->Internal->Listener->Start();
}

//----------------------------------------------------------------------------
void vtkMapServerSource::Stop()
{
  this->Internal->Listener->Stop();
}

//----------------------------------------------------------------------------
void vtkMapServerSource::Poll()
{
  if (this->Internal->Listener->CheckForNewData())
    {
    this->Modified();
    }
}

//-----------------------------------------------------------------------------
void vtkMapServerSource::SetEdgeAngleThreshold(double threshold)
{
  if (threshold == this->GetEdgeAngleThreshold())
    {
      return;
    }

  this->Internal->Listener->SetEdgeAngleThreshold(threshold);
  this->Modified();
}

//-----------------------------------------------------------------------------
double vtkMapServerSource::GetEdgeAngleThreshold()
{
  return this->Internal->Listener->GetEdgeAngleThreshold();
}

//-----------------------------------------------------------------------------
int vtkMapServerSource::GetNumberOfDatasets(int viewId)
{
  this->Internal->Listener->SetDistanceRange(this->DistanceRange);
  return this->Internal->Listener->GetDatasets(viewId).size();
}

//-----------------------------------------------------------------------------
vtkIdType vtkMapServerSource::GetCurrentMapId(int viewId)
{
  this->Internal->Listener->SetDistanceRange(this->DistanceRange);
  return this->Internal->Listener->GetCurrentMapId(viewId);
}

//-----------------------------------------------------------------------------
void vtkMapServerSource::GetDataForMapId(int viewId, vtkIdType mapId, vtkPolyData* polyData)
{
  this->Internal->Listener->SetDistanceRange(this->DistanceRange);
  this->Internal->Listener->GetDataForMapId(viewId, mapId, polyData);
}

//-----------------------------------------------------------------------------
void vtkMapServerSource::GetMeshForMapId(int viewId, vtkIdType mapId, vtkPolyData* polyData)
{
  this->Internal->Listener->SetDistanceRange(this->DistanceRange);
  this->Internal->Listener->GetMeshForMapId(viewId, mapId, polyData);
}

//-----------------------------------------------------------------------------
void vtkMapServerSource::GetDataForMapId(int viewId, vtkIdType mapId, vtkImageData* imageData, vtkTransform* transform)
{
  this->Internal->Listener->SetDistanceRange(this->DistanceRange);
  this->Internal->Listener->GetDataForMapId(viewId, mapId, imageData, transform);
}

vtkIdType vtkMapServerSource::GetLastScanBundleUTime()
{
  return this->Internal->Listener->GetLastScanBundleUTime();
}

//-----------------------------------------------------------------------------
vtkPolyData* vtkMapServerSource::GetDataset(int viewId, vtkIdType i)
{
  this->Internal->Listener->SetDistanceRange(this->DistanceRange);
  return this->Internal->Listener->GetDatasetForTime(viewId, i);
}

//-----------------------------------------------------------------------------
vtkIntArray* vtkMapServerSource::GetViewIds()
{
  this->Internal->Listener->SetDistanceRange(this->DistanceRange);
  return this->Internal->Listener->GetViewIds();
}

//-----------------------------------------------------------------------------
int vtkMapServerSource::RequestInformation(vtkInformation *request,
                                     vtkInformationVector **inputVector,
                                     vtkInformationVector *outputVector)
{
  vtkInformation *info = outputVector->GetInformationObject(0);


  std::vector<double> timesteps = this->Internal->Listener->GetTimesteps(WORKSPACE_DEPTH_VIEW_ID);
  if (timesteps.size())
    {
    double timeRange[2] = {timesteps.front(), timesteps.back()};

    printf("time range: [%f, %f]\n", timeRange[0], timeRange[1]);

    info->Set(vtkStreamingDemandDrivenPipeline::TIME_STEPS(), &timesteps.front(), timesteps.size());
    info->Set(vtkStreamingDemandDrivenPipeline::TIME_RANGE(), timeRange, 2);
    }
  else
    {
    printf("no timesteps available\n");
    info->Remove(vtkStreamingDemandDrivenPipeline::TIME_STEPS());
    info->Remove(vtkStreamingDemandDrivenPipeline::TIME_RANGE());
    }

  return 1;
}

//----------------------------------------------------------------------------
int vtkMapServerSource::RequestData(
  vtkInformation *vtkNotUsed(request),
  vtkInformationVector **inputVector,
  vtkInformationVector *outputVector)
{
  vtkInformation *info = outputVector->GetInformationObject(0);
  vtkDataSet *output = vtkDataSet::SafeDownCast(info->Get(vtkDataObject::DATA_OBJECT()));

  int timestep = 0;
  if (info->Has(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEPS()))
    {
    double timeRequest = info->Get(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEPS())[0];
    timestep = static_cast<int>(floor(timeRequest+0.5));
    }

  this->Internal->Listener->SetDistanceRange(this->DistanceRange);
  vtkSmartPointer<vtkPolyData> polyData = this->Internal->Listener->GetDatasetForTime(WORKSPACE_DEPTH_VIEW_ID, timestep);
  if (polyData)
    {
    output->ShallowCopy(polyData);
    }
  else
    {
    printf("no map data for timestep: %d\n", timestep);
    }

  return 1;
}

//----------------------------------------------------------------------------
void vtkMapServerSource::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
}
