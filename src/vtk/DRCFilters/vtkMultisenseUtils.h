#ifndef __vtkMultisenseUtils_h
#define __vtkMultisenseUtils_h


#include <cstdio>
#include <iostream>
#include <string>

#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>

#include <lcm/lcm-cpp.hpp>

#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>

#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/multisense.hpp>



#include <vtkSmartPointer.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkFloatArray.h>
#include <vtkUnsignedIntArray.h>
#include <vtkIdTypeArray.h>
#include <vtkCellArray.h>
#include <vtkTimerLog.h>


//----------------------------------------------------------------------------
namespace
{


class ScanLineData
{
public:

  uint64_t Revolution;
  uint64_t ScanLineId;
  double SpindleAngle;
  Eigen::Isometry3d ScanToLocalStart;
  Eigen::Isometry3d ScanToLocalEnd;
  bot_core::planar_lidar_t msg;
};


class DataArrays
{
public:

  vtkSmartPointer<vtkPolyData> Dataset;

  vtkPoints* Points;
  vtkFloatArray* Intensity;
  vtkUnsignedIntArray* ScanLineId;
  vtkFloatArray* Azimuth;
  vtkFloatArray* SpindleAngle;
  vtkFloatArray* Distance;
  vtkFloatArray* ZHeight;
  vtkFloatArray* ScanDelta;
  vtkUnsignedIntArray* Timestamp;
};

//----------------------------------------------------------------------------
vtkSmartPointer<vtkCellArray> NewVertexCells(vtkIdType numberOfVerts)
{
  vtkSmartPointer<vtkIdTypeArray> cells = vtkSmartPointer<vtkIdTypeArray>::New();
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

//-----------------------------------------------------------------------------
DataArrays CreateData(vtkIdType numberOfPoints)
{
  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();

  // points
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  points->SetDataTypeToFloat();
  points->Allocate(numberOfPoints);
  polyData->SetPoints(points.GetPointer());
  polyData->SetVerts(NewVertexCells(numberOfPoints));

  // intensity
  vtkSmartPointer<vtkFloatArray> intensity = vtkSmartPointer<vtkFloatArray>::New();
  intensity->SetName("intensity");
  intensity->Allocate(numberOfPoints);
  polyData->GetPointData()->AddArray(intensity.GetPointer());

  // scan line id
  vtkSmartPointer<vtkUnsignedIntArray> scanLineId = vtkSmartPointer<vtkUnsignedIntArray>::New();
  scanLineId->SetName("scan_line_id");
  scanLineId->Allocate(numberOfPoints);
  polyData->GetPointData()->AddArray(scanLineId.GetPointer());

  // azimuth
  vtkSmartPointer<vtkFloatArray> azimuth = vtkSmartPointer<vtkFloatArray>::New();
  azimuth->SetName("azimuth");
  azimuth->Allocate(numberOfPoints);
  polyData->GetPointData()->AddArray(azimuth.GetPointer());

  // spindle angle
  vtkSmartPointer<vtkFloatArray> spindleAngle = vtkSmartPointer<vtkFloatArray>::New();
  spindleAngle->SetName("spindle_angle");
  spindleAngle->Allocate(numberOfPoints);
  polyData->GetPointData()->AddArray(spindleAngle.GetPointer());

  // range
  vtkSmartPointer<vtkFloatArray> distance = vtkSmartPointer<vtkFloatArray>::New();
  distance->SetName("distance");
  distance->Allocate(numberOfPoints);
  polyData->GetPointData()->AddArray(distance.GetPointer());

  // z
  vtkSmartPointer<vtkFloatArray> zheight = vtkSmartPointer<vtkFloatArray>::New();
  zheight->SetName("z");
  zheight->Allocate(numberOfPoints);
  polyData->GetPointData()->AddArray(zheight.GetPointer());

  // scan delta
  vtkSmartPointer<vtkFloatArray> scanDelta = vtkSmartPointer<vtkFloatArray>::New();
  scanDelta->SetName("scan_delta");
  scanDelta->Allocate(numberOfPoints);
  polyData->GetPointData()->AddArray(scanDelta.GetPointer());

  // timestamp
  vtkSmartPointer<vtkUnsignedIntArray> timestamp = vtkSmartPointer<vtkUnsignedIntArray>::New();
  timestamp->SetName("timestamp");
  timestamp->Allocate(numberOfPoints);
  polyData->GetPointData()->AddArray(timestamp.GetPointer());

  DataArrays arrays;
  arrays.Dataset = polyData;
  arrays.Points = points.GetPointer();
  arrays.Intensity = intensity.GetPointer();
  arrays.ScanLineId = scanLineId.GetPointer();
  arrays.Azimuth = azimuth.GetPointer();
  arrays.SpindleAngle = spindleAngle.GetPointer();
  arrays.Distance = distance.GetPointer();
  arrays.ZHeight = zheight.GetPointer();
  arrays.ScanDelta = scanDelta.GetPointer();
  arrays.Timestamp = timestamp.GetPointer();

  return arrays;
}


void AddScanLine(const ScanLineData& scanLine, DataArrays& dataArrays, double distanceRange[2], double edgeAngleThreshold)
{

  const bot_core::planar_lidar_t* msg = &scanLine.msg;

  double spindleAngle = scanLine.SpindleAngle;
  unsigned int scanLineId = scanLine.ScanLineId;
  double theta = msg->rad0;
  const double thetaStep = msg->radstep;
  const int numPoints = msg->nranges;
  const double edgeFilterEnabledRange = 2.0;
  const std::vector<float>&  ranges = msg->ranges;
  const std::vector<float>& intensities = msg->intensities;

  if (numPoints < 2)
    {
    return;
    }

  double t = 0.0;
  const double tStep = 1.0/(numPoints-1);
  Eigen::Quaterniond q0(scanLine.ScanToLocalStart.linear());
  Eigen::Quaterniond q1(scanLine.ScanToLocalEnd.linear());
  Eigen::Vector3d pos0(scanLine.ScanToLocalStart.translation());
  Eigen::Vector3d pos1(scanLine.ScanToLocalEnd.translation());

  const bool doFilter = (edgeAngleThreshold > 0);
  const float angleThresh = edgeAngleThreshold*M_PI/180;

  float prevDelta = 0;
  float prevScanDelta = 0;
  float prevRange = -1;
  for (int i = 0; i < numPoints; ++i, t += tStep, theta += thetaStep)
  {
    if (ranges[i] < distanceRange[0] || ranges[i] > distanceRange[1])
    {
      continue;
    }


    // Edge effect filter
    if ( (i > 0) && ( i+1 < numPoints) && doFilter)
    {
      float theta1 = msg->rad0 + (i-1)*msg->radstep;
      float theta2 = msg->rad0 + i*msg->radstep;
      float theta3 = msg->rad0 + (i+1)*msg->radstep;
      float r1 = ranges[i-1];
      float r2 = ranges[i];
      float r3 = ranges[i+1];
      Eigen::Vector2f p1(r1*std::cos(theta1), r1*std::sin(theta1));
      Eigen::Vector2f p2(r2*std::cos(theta2), r2*std::sin(theta2));
      Eigen::Vector2f p3(r3*std::cos(theta3), r3*std::sin(theta3));
      Eigen::Vector2f pointDelta1 = (p1-p2).normalized();
      Eigen::Vector2f pointDelta2 = (p3-p2).normalized();
      Eigen::Vector2f ray = p2.normalized();
      float angle1 = std::acos(ray.dot(pointDelta1));
      if (angle1 > M_PI/2) angle1 = M_PI-angle1;
      float angle2 = std::acos(ray.dot(pointDelta2));
      if (angle2 > M_PI/2) angle2 = M_PI-angle2;
      if ((angle1 < angleThresh) && (angle2 < angleThresh)) {
        continue;
      }
    }

    float curRange = ranges[i];
    float curDelta = (prevRange >= 0) ? (ranges[i] - prevRange) : 0;
    prevScanDelta = (std::abs(prevDelta) > std::abs(curDelta)) ?
      -prevDelta : curDelta;
    prevDelta = curDelta;
    prevRange = curRange;

    Eigen::Vector3d pt(ranges[i] * cos(theta), ranges[i] * sin(theta), 0.0);

    Eigen::Quaterniond q = q0.slerp(t, q1);
    Eigen::Vector3d pos = (1-t)*pos0 + t*pos1;
    pt = q*pt + pos;

    dataArrays.Points->InsertNextPoint(pt[0], pt[1], pt[2]);

    dataArrays.Intensity->InsertNextValue(intensities[i]);
    dataArrays.ScanLineId->InsertNextValue(scanLineId);
    dataArrays.Azimuth->InsertNextValue(theta);
    dataArrays.SpindleAngle->InsertNextValue(spindleAngle);
    dataArrays.Distance->InsertNextValue(ranges[i]);
    dataArrays.ZHeight->InsertNextValue(pt[2]);
    dataArrays.ScanDelta->InsertNextValue(0);
    int numValues = dataArrays.ScanDelta->GetNumberOfTuples();
    if (numValues > 1) {
      dataArrays.ScanDelta->SetValue(numValues-2, prevScanDelta);
    }
    dataArrays.Timestamp->InsertNextValue(msg->utime);
  }

}

vtkSmartPointer<vtkPolyData> GetPointCloudFromScanLines(const std::vector<ScanLineData>& scanLines, double distanceRange[2], double edgeAngleThreshold)
{
  //printf("GetPointCloud, given %d scan lines\n", scanLines.size());
  DataArrays dataArrays = CreateData(800 * scanLines.size());

  for (size_t i = 0; i < scanLines.size(); ++i)
  {
    AddScanLine(scanLines[i], dataArrays, distanceRange, edgeAngleThreshold);
  }

  dataArrays.Dataset->SetVerts(NewVertexCells(dataArrays.Dataset->GetNumberOfPoints()));

  //printf("%.1f points per scan line\n", static_cast<float>(dataArrays.Dataset->GetNumberOfPoints())/scanLines.size());

  return dataArrays.Dataset;
}


} // end namespace



#endif
