/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkLidarSource.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkLidarSource.h"

#include "vtkPolyData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkSmartPointer.h"
#include "vtkNew.h"
#include "vtkDoubleArray.h"
#include "vtkTransform.h"
#include "vtkMatrix4x4.h"
#include "vtkMath.h"
#include "vtkSetGet.h"

#include <Eigen/Dense>
#include <lcm/lcm-cpp.hpp>
#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>
#include <lcmtypes/bot_core/planar_lidar_t.hpp>

#include <sys/select.h>
#include <queue>
#include <deque>
#include <mutex>
#include <thread>
#include <functional>
#include <condition_variable>

#include "vtkMultisenseUtils.h"

//----------------------------------------------------------------------------
namespace
{

template<typename T>
class SynchronizedQueue
{
  public:

    SynchronizedQueue () :
      queue_(), mutex_(), cond_(), request_to_end_(false), enqueue_data_(true) { }

    void
    enqueue (const T& data)
    {
      std::unique_lock<std::mutex> lock (mutex_);

      if (enqueue_data_)
      {
        queue_.push (data);
        cond_.notify_one ();
      }
    }

    bool
    dequeue (T& result)
    {
      std::unique_lock<std::mutex> lock (mutex_);

      while (queue_.empty () && (!request_to_end_))
      {
        cond_.wait (lock);
      }

      if (request_to_end_)
      {
        doEndActions ();
        return false;
      }

      result = queue_.front ();
      queue_.pop ();

      return true;
    }

    void
    stopQueue ()
    {
      std::unique_lock<std::mutex> lock (mutex_);
      request_to_end_ = true;
      cond_.notify_one ();
    }

    unsigned int
    size ()
    {
      std::unique_lock<std::mutex> lock (mutex_);
      return static_cast<unsigned int> (queue_.size ());
    }

    bool
    isEmpty () const
    {
      std::unique_lock<std::mutex> lock (mutex_);
      return (queue_.empty ());
    }

  private:
    void
    doEndActions ()
    {
      enqueue_data_ = false;

      while (!queue_.empty ())
      {
        queue_.pop ();
      }
    }

    std::queue<T> queue_;              // Use STL queue to store data
    mutable std::mutex mutex_;       // The mutex to synchronise on
    std::condition_variable cond_;   // The condition to wait for

    bool request_to_end_;
    bool enqueue_data_;
};


//----------------------------------------------------------------------------
class LCMListener
{
public:

  LCMListener()
  {
    this->ShouldStop = true;
    this->NewData = false;
    this->CurrentRevolution = 0;
    this->CurrentScanLine = 0;
    this->SplitAngle = 0;
    this->SplitRange = 180;
    this->LastOffsetSpindleAngle = 0;
    this->MaxNumberOfScanLines = 10000;
    this->botparam_ = 0;
    this->botframes_ = 0;

    this->SweepPolyDataRevolution = -1;
    this->SweepPolyData = 0;

    this->DistanceRange[0] = 0.0;
    this->DistanceRange[1] = 80.0;
    // Used to filter out range returns which are oblique to lidar sensor
    this->EdgeAngleThreshold = 0;  // degrees, 30 was  default

    this->HeightRange[0] = -80.0;
    this->HeightRange[1] = 80.0;

    this->LCMHandle = std::shared_ptr<lcm::LCM>(new lcm::LCM);
    if(!this->LCMHandle->good())
    {
      std::cerr <<"ERROR: lcm is not good()" <<std::endl;
    }

    this->LCMHandle->subscribe("SICK_SCAN", &LCMListener::lidarHandler, this);
  }


  void lidarHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const bot_core::planar_lidar_t* msg)
  {
    this->HandleNewData(msg);
  }

  void InitBotConfig(const char* filename)
  {
    bool useBotParamFromFile = true;

    if (filename && filename[0])
      {
      botparam_ = bot_param_new_from_file(filename);
      }
    else
      {
      while (!botparam_)
        {
        botparam_ = bot_param_new_from_server(this->LCMHandle->getUnderlyingLCM(), 0);
        }
      }

    botframes_ = bot_frames_get_global(this->LCMHandle->getUnderlyingLCM(), botparam_);
  }

  bool CheckForNewData()
  {
    std::lock_guard<std::mutex> lock(this->Mutex);
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
    this->Thread = std::shared_ptr<std::thread>(
      new std::thread(std::bind(&LCMListener::ThreadLoopWithSelect, this)));

    this->SweepThread = std::shared_ptr<std::thread>(
      new std::thread(std::bind(&LCMListener::SweepThreadLoop, this)));
  }

  void Stop()
  {
    if (this->Thread)
      {
      this->ShouldStop = true;
      this->Condition.notify_one();
      this->Thread->join();
      this->Thread.reset();
      this->SweepThread->join();
      this->SweepThread.reset();
      }
  }

  std::vector<double> GetTimesteps()
  {
    std::vector<double> timesteps;
    for (int i = 0; i < this->CurrentRevolution; ++i)
      {
      timesteps.push_back(i);
      }
    return timesteps;
  }

  int GetCurrentRevolution()
  {
    return this->SweepPolyDataRevolution+1;
  }

  int GetCurrentScanLine()
  {
    return this->CurrentScanLine;
  }

  void GetScanLine(std::vector<ScanLineData>& scanLines, int scanLine)
  {
    std::lock_guard<std::mutex> lock(this->Mutex);
    for (std::deque<ScanLineData>::const_iterator itr = this->ScanLines.begin(); itr != this->ScanLines.end(); ++itr)
    {
      if (itr->ScanLineId == scanLine)
      {
        scanLines.push_back(*itr);
        return;
      }
    }
  }

  void GetScanLinesForRevolution(std::vector<ScanLineData>& scanLines, int revolution)
  {
    std::lock_guard<std::mutex> lock(this->Mutex);
    for (std::deque<ScanLineData>::const_iterator itr = this->ScanLines.begin(); itr != this->ScanLines.end(); ++itr)
    {
      if (itr->Revolution == revolution)
      {
        scanLines.push_back(*itr);
      }
    }
  }

  void GetScanLinesForHistory(std::vector<ScanLineData>& scanLines, int numberOfScanLines)
  {
    std::lock_guard<std::mutex> lock(this->Mutex);
    int counter = 0;
    for (std::deque<ScanLineData>::const_iterator itr = this->ScanLines.end(); itr != this->ScanLines.begin(); --itr)
    {
      counter++;
      scanLines.push_back(*itr);

      if (counter > numberOfScanLines)
        return;
    }
  }

  vtkSmartPointer<vtkPolyData> GetDataForScanLine(int scanLine)
  {
    std::vector<ScanLineData> scanLines;
    this->GetScanLine(scanLines, scanLine);

    return GetPointCloudFromScanLines(scanLines, this->DistanceRange, this->EdgeAngleThreshold, this->HeightRange);
  }

  vtkSmartPointer<vtkPolyData> GetDataForRevolution(int revolution)
  {
    if (revolution == this->SweepPolyDataRevolution)
    {
      return this->SweepPolyData;
    }

    std::vector<ScanLineData> scanLines;
    this->GetScanLinesForRevolution(scanLines, revolution);

    return GetPointCloudFromScanLines(scanLines, this->DistanceRange, this->EdgeAngleThreshold, this->HeightRange);
  }

  vtkSmartPointer<vtkPolyData> GetDataForHistory(int numberOfScanLines)
  {
    //if (revolution == this->SweepPolyDataRevolution)
    //{
    //  return this->SweepPolyData;
    //}

    std::vector<ScanLineData> scanLines;
    this->GetScanLinesForHistory(scanLines, numberOfScanLines);

    return GetPointCloudFromScanLines(scanLines, this->DistanceRange, this->EdgeAngleThreshold, this->HeightRange);
  }

  int get_trans_with_utime(std::string from_frame, std::string to_frame,
                                 vtkIdType utime, Eigen::Isometry3d & mat)
  {
    if (!botframes_)
    {
      std::cout << "vtkLidarSource::LCMListener: botframe is not initialized" << std::endl;
      mat = mat.matrix().Identity();
      return 0;
    }

    int status;
    double matx[16];
    status = bot_frames_get_trans_mat_4x4_with_utime( botframes_, from_frame.c_str(),  to_frame.c_str(), utime, matx);
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        mat(i,j) = matx[i*4+j];
      }
    }
    return status;
  }

  void SetDistanceRange(double distanceRange[2])
  {
    this->DistanceRange[0] = distanceRange[0];
    this->DistanceRange[1] = distanceRange[1];
  }

  void SetHeightRange(double heightRange[2])
  {
    this->HeightRange[0] = heightRange[0];
    this->HeightRange[1] = heightRange[1];
  }  

  void SetEdgeAngleThreshold(double edgeAngleThreshold)
  {
    this->EdgeAngleThreshold = edgeAngleThreshold;
  }

  double GetEdgeAngleThreshold()
  {
    return this->EdgeAngleThreshold;
  }

  vtkIdType GetCurrentScanTime()
  {
    return this->CurrentScanTime;
  }

  void SweepThreadLoop()
  {
    while (!this->ShouldStop)
      {
      std::unique_lock<std::mutex> lock(this->SweepMutex);

      this->Condition.wait(lock);
      if (this->ShouldStop)
        {
        break;
        }

      this->SweepPolyData = this->GetDataForRevolution(this->CurrentRevolution-1);
      this->SweepPolyDataRevolution = this->CurrentRevolution-1;
      }
  }

protected:

  void UpdateDequeSize()
  {
    if (this->MaxNumberOfScanLines <= 0)
      {
      return;
      }
    while (this->ScanLines.size() >= this->MaxNumberOfScanLines)
      {
      this->ScanLines.pop_front();
      }
  }

  double MyClamp(double value, double clampRange)
  {
    value = fmod(value, clampRange);
    value = value < 0 ? value + clampRange : value;
    return value;
  }

  void HandleNewData(const bot_core::planar_lidar_t* msg)
  {
    this->CurrentScanTime = msg->utime;

    Eigen::Isometry3d scanToLocalStart;
    Eigen::Isometry3d scanToLocalEnd;
    Eigen::Isometry3d bodyToLocalStart;


    get_trans_with_utime("SICK_SCAN", "local", msg->utime, scanToLocalStart);
    get_trans_with_utime("SICK_SCAN", "local", msg->utime +  1e6*3/(40*4), scanToLocalEnd);

    get_trans_with_utime("body", "local", msg->utime, bodyToLocalStart);

    Eigen::Isometry3d spindleRotation;
    get_trans_with_utime("PRE_SPINDLE", "POST_SPINDLE", msg->utime, spindleRotation);

    Eigen::Matrix3d rot = spindleRotation.rotation();
    Eigen::Vector3d eulerAngles = rot.eulerAngles(0, 1, 2);
    //printf("euler angles: %f %f %f\n", eulerAngles[0], eulerAngles[1], eulerAngles[2]);

    double spindleAngle = eulerAngles[2] * (180.0 / M_PI) + 180.0;
    spindleAngle = MyClamp(spindleAngle, 360.0);
    spindleAngle = 360 - spindleAngle;

    //printf("scan line at angle, rev: %f, %d\n", spindleAngle, this->CurrentRevolution);


    double offsetSpindleAngle = spindleAngle - this->SplitAngle;
    offsetSpindleAngle = MyClamp(offsetSpindleAngle, this->SplitRange);

    //printf("offset spindle angle: %f --> %f\n", spindleAngle, offsetSpindleAngle);

    if (offsetSpindleAngle < this->LastOffsetSpindleAngle)
    {
      //printf("---> splitting revolution %d at angle %f, total scan lines: %d\n", this->CurrentRevolution, spindleAngle, this->ScanLines.size());
      this->CurrentRevolution++;
      this->NewData = true;
      this->Condition.notify_one();
    }

    this->LastOffsetSpindleAngle = offsetSpindleAngle;

    std::lock_guard<std::mutex> lock(this->Mutex);

    this->ScanLines.resize(this->ScanLines.size() + 1);
    ScanLineData& scanLine = this->ScanLines.back();
    scanLine.ScanLineId = this->CurrentScanLine++;
    scanLine.ScanToLocalStart = scanToLocalStart;
    scanLine.ScanToLocalEnd = scanToLocalEnd;
    scanLine.BodyToLocalStart = bodyToLocalStart;
    scanLine.SpindleAngle = spindleAngle;
    scanLine.Revolution = this->CurrentRevolution;
    scanLine.msg = *msg;

    this->UpdateDequeSize();
  }

  bool NewData;
  bool ShouldStop;
  int MaxNumberOfScanLines;
  int CurrentRevolution;
  int CurrentScanLine;

  double SplitAngle;
  double SplitRange;
  double LastOffsetSpindleAngle;

  double EdgeAngleThreshold;
  double DistanceRange[2];
  double HeightRange[2];

  vtkSmartPointer<vtkPolyData> SweepPolyData;
  int SweepPolyDataRevolution;

  std::mutex Mutex;
  std::mutex SweepMutex;
  std::condition_variable Condition;

  std::deque<ScanLineData> ScanLines;

  std::shared_ptr<lcm::LCM> LCMHandle;

  std::shared_ptr<std::thread> Thread;
  std::shared_ptr<std::thread> SweepThread;

  vtkIdType CurrentScanTime;

  BotParam* botparam_;
  BotFrames* botframes_;

};


} // end namespace

//----------------------------------------------------------------------------
class vtkLidarSource::vtkInternal
{
public:

  vtkInternal()
  {
    this->Listener = std::shared_ptr<LCMListener>(new LCMListener);
  }

  ~vtkInternal()
  {
  }

  std::shared_ptr<LCMListener> Listener;
};

//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkLidarSource);

//----------------------------------------------------------------------------
vtkLidarSource::vtkLidarSource()
{
  this->Internal = new vtkInternal;
  this->SetNumberOfInputPorts(0);
  this->SetNumberOfOutputPorts(1);

  this->DistanceRange[0] = 0.0;
  this->DistanceRange[1] = 80.0;
  this->Internal->Listener->SetDistanceRange(this->DistanceRange);
  this->HeightRange[0] = -80.0;
  this->HeightRange[1] = 80.0;
  this->Internal->Listener->SetHeightRange(this->HeightRange);

}

//----------------------------------------------------------------------------
vtkLidarSource::~vtkLidarSource()
{
  this->Stop();
  delete this->Internal;
}

//----------------------------------------------------------------------------
void vtkLidarSource::Start()
{
  this->Internal->Listener->Start();
}

//----------------------------------------------------------------------------
void vtkLidarSource::Stop()
{
  this->Internal->Listener->Stop();
}

//----------------------------------------------------------------------------
void vtkLidarSource::Poll()
{
  if (this->Internal->Listener->CheckForNewData())
    {
    this->Modified();
    }
}

//-----------------------------------------------------------------------------
void vtkLidarSource::InitBotConfig(const char* filename)
{
  this->Internal->Listener->InitBotConfig(filename);
}

//-----------------------------------------------------------------------------
void vtkLidarSource::GetBotRollPitchYaw(vtkTransform* transform, double rpy[3])
{
  double angleAxis[4];
  transform->GetOrientationWXYZ(angleAxis);
  angleAxis[0] = vtkMath::RadiansFromDegrees(angleAxis[0]);

  bot_angle_axis_to_roll_pitch_yaw (angleAxis[0], angleAxis+1, rpy);
}

//-----------------------------------------------------------------------------
void vtkLidarSource::GetBotQuaternion(vtkTransform* transform, double wxyz[4])
{
  double angleAxis[4];
  transform->GetOrientationWXYZ(angleAxis);
  angleAxis[0] = vtkMath::RadiansFromDegrees(angleAxis[0]);

  bot_angle_axis_to_quat(angleAxis[0], angleAxis+1, wxyz);
}

//-----------------------------------------------------------------------------
void vtkLidarSource::GetTransform(const char* fromFrame, const char* toFrame, vtkIdType utime, vtkTransform* transform)
{
  if (!transform)
    {
    return;
    }

  transform->Identity();

  Eigen::Isometry3d mat;
  this->Internal->Listener->get_trans_with_utime(fromFrame, toFrame, utime, mat);

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
void vtkLidarSource::SetEdgeAngleThreshold(double threshold)
{
  if (threshold == this->GetEdgeAngleThreshold())
    {
      return;
    }

  this->Internal->Listener->SetEdgeAngleThreshold(threshold);
  this->Modified();
}

//-----------------------------------------------------------------------------
double vtkLidarSource::GetEdgeAngleThreshold()
{
  return this->Internal->Listener->GetEdgeAngleThreshold();
}

//-----------------------------------------------------------------------------
vtkIdType vtkLidarSource::GetCurrentScanTime()
{
  return this->Internal->Listener->GetCurrentScanTime();
}

//-----------------------------------------------------------------------------
int vtkLidarSource::GetCurrentRevolution()
{
  this->Internal->Listener->SetDistanceRange(this->DistanceRange);
  this->Internal->Listener->SetHeightRange(this->HeightRange);
  return this->Internal->Listener->GetCurrentRevolution();
}

//-----------------------------------------------------------------------------
int vtkLidarSource::GetCurrentScanLine()
{
  this->Internal->Listener->SetDistanceRange(this->DistanceRange);
  this->Internal->Listener->SetHeightRange(this->HeightRange);
  return this->Internal->Listener->GetCurrentScanLine();
}

//-----------------------------------------------------------------------------
void vtkLidarSource::GetDataForRevolution(int revolution, vtkPolyData* polyData)
{
  this->Internal->Listener->SetDistanceRange(this->DistanceRange);
  this->Internal->Listener->SetHeightRange(this->HeightRange);
  vtkSmartPointer<vtkPolyData> data = this->Internal->Listener->GetDataForRevolution(revolution);
  polyData->ShallowCopy(data);
}

//-----------------------------------------------------------------------------
void vtkLidarSource::GetDataForHistory(int numberOfScanLines, vtkPolyData* polyData)
{
  this->Internal->Listener->SetDistanceRange(this->DistanceRange);
  this->Internal->Listener->SetHeightRange(this->HeightRange);
  vtkSmartPointer<vtkPolyData> data = this->Internal->Listener->GetDataForHistory(numberOfScanLines);
  polyData->ShallowCopy(data);
}

//-----------------------------------------------------------------------------
void vtkLidarSource::GetDataForScanLine(int scanLine, vtkPolyData* polyData)
{
  this->Internal->Listener->SetDistanceRange(this->DistanceRange);
  this->Internal->Listener->SetHeightRange(this->HeightRange);
  vtkSmartPointer<vtkPolyData> data = this->Internal->Listener->GetDataForScanLine(scanLine);
  polyData->ShallowCopy(data);
}

//-----------------------------------------------------------------------------
int vtkLidarSource::RequestInformation(vtkInformation *request,
                                     vtkInformationVector **inputVector,
                                     vtkInformationVector *outputVector)
{
  vtkInformation *info = outputVector->GetInformationObject(0);

  std::vector<double> timesteps = this->Internal->Listener->GetTimesteps();
  if (timesteps.size())
    {
    double timeRange[2] = {timesteps.front(), timesteps.back()};
    info->Set(vtkStreamingDemandDrivenPipeline::TIME_STEPS(), &timesteps.front(), timesteps.size());
    info->Set(vtkStreamingDemandDrivenPipeline::TIME_RANGE(), timeRange, 2);
    }
  else
    {
    info->Remove(vtkStreamingDemandDrivenPipeline::TIME_STEPS());
    info->Remove(vtkStreamingDemandDrivenPipeline::TIME_RANGE());
    }

  return 1;
}

//----------------------------------------------------------------------------
int vtkLidarSource::RequestData(
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
  this->Internal->Listener->SetHeightRange(this->HeightRange);
  vtkSmartPointer<vtkPolyData> polyData = this->Internal->Listener->GetDataForRevolution(timestep);
  if (polyData)
    {
    output->ShallowCopy(polyData);
    }

  return 1;
}

//----------------------------------------------------------------------------
void vtkLidarSource::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
}
