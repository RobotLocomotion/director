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
#include "vtkNew.h"


#include <vtkIdList.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>


#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>

#include <lcm/lcm-cpp.hpp>

//#include <lcmtypes/drc_lcmtypes.hpp>
#include <lcmtypes/drc/map_image_t.hpp>


#include <maps/LcmTranslator.hpp>
#include <maps/DepthImageView.hpp>
#include <maps/PointCloudView.hpp>


#include <sys/select.h>
#include <deque>

namespace
{

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
class MapData
{
public:
  uint64_t Id;
  vtkSmartPointer<vtkPolyData> Data;
};


//----------------------------------------------------------------------------
class LCMListener
{
public:

  LCMListener()
  {
    this->ShouldStop = true;
    this->NewData = false;
    this->MaxNumberOfDatasets = 100;
    this->CurrentMapId = -1;

    this->LCMHandle = boost::shared_ptr<lcm::LCM>(new lcm::LCM);
    if(!this->LCMHandle->good())
    {
      std::cerr <<"ERROR: lcm is not good()" <<std::endl;
    }

    this->LCMHandle->subscribe( "MAP_DEPTH", &LCMListener::lidarHandler, this);
  }


  void lidarHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const drc::map_image_t* msg)
  {
    this->HandleNewData(msg);
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
    return (status != 0 && FD_ISSET(lcmFd, &fds));
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

    printf("starting\n");

    this->ShouldStop = false;
    this->Thread = boost::shared_ptr<boost::thread>(
      new boost::thread(boost::bind(&LCMListener::ThreadLoopWithSelect, this)));
  }

  void Stop()
  {
    if (this->Thread)
      {
      printf("stopping thread...\n");
      this->ShouldStop = true;
      this->Thread->interrupt();
      this->Thread->join();
      this->Thread.reset();
      printf("done.\n");
      }
  }

  std::vector<vtkSmartPointer<vtkPolyData> > GetDatasets()
  {
    std::vector<vtkSmartPointer<vtkPolyData> > datasets;

    boost::lock_guard<boost::mutex> lock(this->Mutex);
    for (size_t i = 0; i < this->Datasets.size(); ++i)
      {
      datasets.push_back(this->Datasets[i].Data);
      }

    return datasets;
  }

  std::vector<double> GetTimesteps()
  {
    boost::lock_guard<boost::mutex> lock(this->Mutex);

    std::vector<double> timesteps;
    for (int i = 0; i < this->Datasets.size(); ++i)
      {
      timesteps.push_back(i);
      }
    return timesteps;
  }

  vtkSmartPointer<vtkPolyData> GetDatasetForTime(int timestep)
  {
    printf("getting data for timestep: %d\n", timestep);

    boost::lock_guard<boost::mutex> lock(this->Mutex);
    if (timestep >= 0 && timestep < this->Datasets.size())
      {
      return this->Datasets[timestep].Data;
      }
    else
      {
      return 0;
      }
  }

  vtkIdType GetCurrentMapId()
  {
    return this->CurrentMapId;
  }

  void GetDataForMapId(vtkIdType mapId, vtkPolyData* polyData)
  {

  }

protected:

  void UpdateDequeSize()
  {
    if (this->MaxNumberOfDatasets <= 0)
      {
      return;
      }
    while (this->Datasets.size() >= this->MaxNumberOfDatasets)
      {
      this->Datasets.pop_front();
      }
  }

  void HandleNewData(const drc::map_image_t* msg)
  {
    maps::DepthImageView depthImage;
    maps::LcmTranslator::fromLcm(*msg, depthImage);

    maps::PointCloud::Ptr pointCloud = depthImage.getAsPointCloud();

    MapData mapData;
    mapData.Id = ++this->CurrentMapId;
    mapData.Data = PolyDataFromPointCloud(pointCloud);

    // store data
    boost::lock_guard<boost::mutex> lock(this->Mutex);
    this->Datasets.push_back(mapData);
    this->UpdateDequeSize();
    this->NewData = true;
  }

  bool NewData;
  bool ShouldStop;
  int MaxNumberOfDatasets;
  vtkIdType CurrentMapId;

  boost::mutex Mutex;

  std::deque<MapData> Datasets;

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
int vtkMapServerSource::GetNumberOfDatasets()
{
  return this->Internal->Listener->GetDatasets().size();
}

//-----------------------------------------------------------------------------
vtkIdType vtkMapServerSource::GetCurrentMapId()
{
  return this->Internal->Listener->GetCurrentMapId();
}

//-----------------------------------------------------------------------------
void vtkMapServerSource::GetDataForMapId(vtkIdType mapId, vtkPolyData* polyData)
{
  return this->Internal->Listener->GetDataForMapId(mapId, polyData);
}

//-----------------------------------------------------------------------------
vtkPolyData* vtkMapServerSource::GetDataset(int i)
{
  return this->Internal->Listener->GetDatasetForTime(i);
}

//-----------------------------------------------------------------------------
int vtkMapServerSource::RequestInformation(vtkInformation *request,
                                     vtkInformationVector **inputVector,
                                     vtkInformationVector *outputVector)
{
  vtkInformation *info = outputVector->GetInformationObject(0);

  printf("requst info\n");

  std::vector<double> timesteps = this->Internal->Listener->GetTimesteps();
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
  printf("request data\n");

  vtkInformation *info = outputVector->GetInformationObject(0);
  vtkDataSet *output = vtkDataSet::SafeDownCast(info->Get(vtkDataObject::DATA_OBJECT()));

  int timestep = 0;
  if (info->Has(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEPS()))
    {
    double timeRequest = info->Get(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEPS())[0];
    timestep = static_cast<int>(floor(timeRequest+0.5));
    }

  vtkSmartPointer<vtkPolyData> polyData = this->Internal->Listener->GetDatasetForTime(timestep);
  if (polyData)
    {
    output->ShallowCopy(polyData);
    }
  else
    {
    printf("no data\n");
    }

  return 1;
}

//----------------------------------------------------------------------------
void vtkMapServerSource::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
}
