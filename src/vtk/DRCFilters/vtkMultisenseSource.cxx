/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkMultisenseSource.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkMultisenseSource.h"


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

#include <boost/thread/thread.hpp>

#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>

#include <lcm/lcm-cpp.hpp>

#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>

#include <lcmtypes/bot_core.hpp>

#include <sys/select.h>

#include <queue>
#include <deque>

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
      boost::unique_lock<boost::mutex> lock (mutex_);

      if (enqueue_data_)
      {
        queue_.push (data);
        cond_.notify_one ();
      }
    }

    bool
    dequeue (T& result)
    {
      boost::unique_lock<boost::mutex> lock (mutex_);

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
      boost::unique_lock<boost::mutex> lock (mutex_);
      request_to_end_ = true;
      cond_.notify_one ();
    }

    unsigned int
    size ()
    {
      boost::unique_lock<boost::mutex> lock (mutex_);
      return static_cast<unsigned int> (queue_.size ());
    }

    bool
    isEmpty () const
    {
      boost::unique_lock<boost::mutex> lock (mutex_);
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
    mutable boost::mutex mutex_;       // The mutex to synchronise on
    boost::condition_variable cond_;   // The condition to wait for

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
    this->DistanceRange[1] = 30.0;
    this->EdgeAngleThreshold = 30;  // degrees

    this->LCMHandle = boost::shared_ptr<lcm::LCM>(new lcm::LCM);
    if(!this->LCMHandle->good())
    {
      std::cerr <<"ERROR: lcm is not good()" <<std::endl;
    }

    this->LCMHandle->subscribe("SCAN", &LCMListener::lidarHandler, this);
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

    this->SweepThread = boost::shared_ptr<boost::thread>(
      new boost::thread(boost::bind(&LCMListener::SweepThreadLoop, this)));
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
    boost::lock_guard<boost::mutex> lock(this->Mutex);
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
    boost::lock_guard<boost::mutex> lock(this->Mutex);
    for (std::deque<ScanLineData>::const_iterator itr = this->ScanLines.begin(); itr != this->ScanLines.end(); ++itr)
    {
      if (itr->Revolution == revolution)
      {
        scanLines.push_back(*itr);
      }
    }
  }

  vtkSmartPointer<vtkPolyData> GetDataForScanLine(int scanLine)
  {
    std::vector<ScanLineData> scanLines;
    this->GetScanLine(scanLines, scanLine);

    return GetPointCloudFromScanLines(scanLines, this->DistanceRange, this->EdgeAngleThreshold);
  }

  vtkSmartPointer<vtkPolyData> GetDataForRevolution(int revolution)
  {
    if (revolution == this->SweepPolyDataRevolution)
    {
      return this->SweepPolyData;
    }

    std::vector<ScanLineData> scanLines;
    this->GetScanLinesForRevolution(scanLines, revolution);

    return GetPointCloudFromScanLines(scanLines, this->DistanceRange, this->EdgeAngleThreshold);
  }

  int get_trans_with_utime(std::string from_frame, std::string to_frame,
                                 vtkIdType utime, Eigen::Isometry3d & mat)
  {
    if (!botframes_)
    {
      std::cout << "vtkMultisenseSource::LCMListener: botframe is not initialized" << std::endl;
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
      boost::unique_lock<boost::mutex> lock(this->SweepMutex);

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

    get_trans_with_utime("SCAN", "local", msg->utime, scanToLocalStart);
    get_trans_with_utime("SCAN", "local", msg->utime +  1e6*3/(40*4), scanToLocalEnd);

    //get_trans_with_utime("SCAN", "PRE_SPINDLE", msg->utime, scanToLocal);

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

    boost::lock_guard<boost::mutex> lock(this->Mutex);

    this->ScanLines.resize(this->ScanLines.size() + 1);
    ScanLineData& scanLine = this->ScanLines.back();
    scanLine.ScanLineId = this->CurrentScanLine++;
    scanLine.ScanToLocalStart = scanToLocalStart;
    scanLine.ScanToLocalEnd = scanToLocalEnd;
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

  vtkSmartPointer<vtkPolyData> SweepPolyData;
  int SweepPolyDataRevolution;

  boost::mutex Mutex;
  boost::mutex SweepMutex;
  boost::condition_variable Condition;

  std::deque<ScanLineData> ScanLines;

  boost::shared_ptr<lcm::LCM> LCMHandle;

  boost::shared_ptr<boost::thread> Thread;
  boost::shared_ptr<boost::thread> SweepThread;

  vtkIdType CurrentScanTime;

  BotParam* botparam_;
  BotFrames* botframes_;

};


} // end namespace

//----------------------------------------------------------------------------
class vtkMultisenseSource::vtkInternal
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
vtkStandardNewMacro(vtkMultisenseSource);

//----------------------------------------------------------------------------
vtkMultisenseSource::vtkMultisenseSource()
{
  this->Internal = new vtkInternal;
  this->SetNumberOfInputPorts(0);
  this->SetNumberOfOutputPorts(1);

  this->DistanceRange[0] = 0.0;
  this->DistanceRange[1] = 30.0;
  this->Internal->Listener->SetDistanceRange(this->DistanceRange);
}

//----------------------------------------------------------------------------
vtkMultisenseSource::~vtkMultisenseSource()
{
  this->Stop();
  delete this->Internal;
}

//----------------------------------------------------------------------------
void vtkMultisenseSource::Start()
{
  this->Internal->Listener->Start();
}

//----------------------------------------------------------------------------
void vtkMultisenseSource::Stop()
{
  this->Internal->Listener->Stop();
}

//----------------------------------------------------------------------------
void vtkMultisenseSource::Poll()
{
  if (this->Internal->Listener->CheckForNewData())
    {
    this->Modified();
    }
}

//-----------------------------------------------------------------------------
void vtkMultisenseSource::InitBotConfig(const char* filename)
{
  this->Internal->Listener->InitBotConfig(filename);
}

//-----------------------------------------------------------------------------
void vtkMultisenseSource::GetBotRollPitchYaw(vtkTransform* transform, double rpy[3])
{
  double angleAxis[4];
  transform->GetOrientationWXYZ(angleAxis);
  angleAxis[0] = vtkMath::RadiansFromDegrees(angleAxis[0]);

  bot_angle_axis_to_roll_pitch_yaw (angleAxis[0], angleAxis+1, rpy);
}

//-----------------------------------------------------------------------------
void vtkMultisenseSource::GetBotQuaternion(vtkTransform* transform, double wxyz[4])
{
  double angleAxis[4];
  transform->GetOrientationWXYZ(angleAxis);
  angleAxis[0] = vtkMath::RadiansFromDegrees(angleAxis[0]);

  bot_angle_axis_to_quat(angleAxis[0], angleAxis+1, wxyz);
}

//-----------------------------------------------------------------------------
void vtkMultisenseSource::GetTransform(const char* fromFrame, const char* toFrame, vtkIdType utime, vtkTransform* transform)
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
void vtkMultisenseSource::SetEdgeAngleThreshold(double threshold)
{
  if (threshold == this->GetEdgeAngleThreshold())
    {
      return;
    }

  this->Internal->Listener->SetEdgeAngleThreshold(threshold);
  this->Modified();
}

//-----------------------------------------------------------------------------
double vtkMultisenseSource::GetEdgeAngleThreshold()
{
  return this->Internal->Listener->GetEdgeAngleThreshold();
}

//-----------------------------------------------------------------------------
vtkIdType vtkMultisenseSource::GetCurrentScanTime()
{
  return this->Internal->Listener->GetCurrentScanTime();
}

//-----------------------------------------------------------------------------
int vtkMultisenseSource::GetCurrentRevolution()
{
  this->Internal->Listener->SetDistanceRange(this->DistanceRange);
  return this->Internal->Listener->GetCurrentRevolution();
}

//-----------------------------------------------------------------------------
int vtkMultisenseSource::GetCurrentScanLine()
{
  this->Internal->Listener->SetDistanceRange(this->DistanceRange);
  return this->Internal->Listener->GetCurrentScanLine();
}

//-----------------------------------------------------------------------------
void vtkMultisenseSource::GetDataForRevolution(int revolution, vtkPolyData* polyData)
{
  this->Internal->Listener->SetDistanceRange(this->DistanceRange);
  vtkSmartPointer<vtkPolyData> data = this->Internal->Listener->GetDataForRevolution(revolution);
  polyData->ShallowCopy(data);
}

//-----------------------------------------------------------------------------
void vtkMultisenseSource::GetDataForScanLine(int scanLine, vtkPolyData* polyData)
{
  this->Internal->Listener->SetDistanceRange(this->DistanceRange);
  vtkSmartPointer<vtkPolyData> data = this->Internal->Listener->GetDataForScanLine(scanLine);
  polyData->ShallowCopy(data);
}

//-----------------------------------------------------------------------------
int vtkMultisenseSource::RequestInformation(vtkInformation *request,
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
int vtkMultisenseSource::RequestData(
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
  vtkSmartPointer<vtkPolyData> polyData = this->Internal->Listener->GetDataForRevolution(timestep);
  if (polyData)
    {
    output->ShallowCopy(polyData);
    }

  return 1;
}

//----------------------------------------------------------------------------
void vtkMultisenseSource::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
}
