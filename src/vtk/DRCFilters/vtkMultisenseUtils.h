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
  Eigen::Isometry3d ScanToLocal;
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
  distance->SetNumberOfTuples(numberOfPoints);
  polyData->GetPointData()->AddArray(distance.GetPointer());

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
  arrays.Timestamp = timestamp.GetPointer();

  return arrays;
}


void AddScanLine(const ScanLineData& scanLine, DataArrays& dataArrays, double distanceRange[2], double edgeDistanceThreshold)
{

  const bot_core::planar_lidar_t* msg = &scanLine.msg;

  double spindleAngle = scanLine.SpindleAngle;
  unsigned int scanLineId = scanLine.ScanLineId;
  double thetaStart = msg->rad0;
  const double thetaStep = msg->radstep;
  const int numPoints = msg->nranges;
  const double edgeFilterEnabledRange = 2.0;
  const std::vector<float>&  ranges = msg->ranges;
  const std::vector<float>& intensities = msg->intensities;

  Eigen::Affine3d aff = Eigen::Affine3d(scanLine.ScanToLocal);
  Eigen::AngleAxisd angleAxis = Eigen::AngleAxisd(spindleAngle, Eigen::Vector3d(0,1,0));


  if (msg->nranges != msg->nintensities)
  {
    printf("ranges and intensities don't match\n");
  }


  for (int i = 0; i < numPoints; ++i)
  {
    if (ranges[i] < distanceRange[0] || ranges[i] > distanceRange[1])
    {
      continue;
    }


    // Edge effect filter
    if ( (i > 0) && ( i+1 < numPoints))
    {
      float rightDiff = fabs(ranges[i] - ranges[i-1]);
      float leftDiff = fabs(ranges[i] - ranges[i+1]);
      if (( rightDiff > edgeDistanceThreshold) || (leftDiff > edgeDistanceThreshold ))
      {
        if (ranges[i] < edgeFilterEnabledRange)
        {
          continue;
        }
      }
    }

    double theta = thetaStart + i*thetaStep;
    Eigen::Vector3d pt(ranges[i] * cos(theta), ranges[i] * sin(theta), 0.0);
    pt = aff * pt;
    //pt = angleAxis.matrix() * pt;

    dataArrays.Points->InsertNextPoint(pt[0], pt[1], pt[2]);

    dataArrays.Intensity->InsertNextValue(intensities[i]);
    dataArrays.ScanLineId->InsertNextValue(scanLineId);
    dataArrays.Azimuth->InsertNextValue(theta);
    dataArrays.SpindleAngle->InsertNextValue(spindleAngle);
    dataArrays.Distance->InsertNextValue(ranges[i]);
    dataArrays.Timestamp->InsertNextValue(msg->utime);
  }

}

vtkSmartPointer<vtkPolyData> GetPointCloudFromScanLines(const std::vector<ScanLineData>& scanLines, double distanceRange[2], double edgeDistanceThreshold)
{
  //printf("GetPointCloud, given %d scan lines\n", scanLines.size());
  DataArrays dataArrays = CreateData(800 * scanLines.size());

  for (size_t i = 0; i < scanLines.size(); ++i)
  {
    AddScanLine(scanLines[i], dataArrays, distanceRange, edgeDistanceThreshold);
  }

  dataArrays.Dataset->SetVerts(NewVertexCells(dataArrays.Dataset->GetNumberOfPoints()));

  //printf("%.1f points per scan line\n", static_cast<float>(dataArrays.Dataset->GetNumberOfPoints())/scanLines.size());

  return dataArrays.Dataset;
}


} // end namespace



#endif
