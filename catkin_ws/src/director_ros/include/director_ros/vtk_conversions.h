#ifndef __vtk_conversions_h
#define __vtk_conversions_h


#include <cstdio>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <vtkPolyData.h>
#include <vtkCellArray.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkIdTypeArray.h>


namespace {

// ----------------------------------------------------------------------------
vtkSmartPointer<vtkCellArray> NewVertexCells(vtkIdType numberOfVerts) {
  vtkSmartPointer<vtkIdTypeArray> cells = vtkSmartPointer<vtkIdTypeArray>::New();
  cells->SetNumberOfValues(numberOfVerts*2);
  vtkIdType* ids = cells->GetPointer(0);
  for (vtkIdType i = 0; i < numberOfVerts; ++i) {
    ids[i*2] = 1;
    ids[i*2+1] = i;
  }

  vtkSmartPointer<vtkCellArray> cellArray = vtkSmartPointer<vtkCellArray>::New();
  cellArray->SetCells(numberOfVerts, cells.GetPointer());
  return cellArray;
}

// ----------------------------------------------------------------------------
vtkPolyData* ConvertPointCloud2ToVtk(const sensor_msgs::PointCloud2Ptr& msg) {

  const size_t numberOfPoints = msg->height*msg->width;

  vtkPolyData* polyData = vtkPolyData::New();
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  points->SetDataTypeToFloat();
  points->Allocate(numberOfPoints);
  polyData->SetPoints(points);
  polyData->SetVerts(NewVertexCells(numberOfPoints));

  vtkSmartPointer<vtkFloatArray> intensity = vtkSmartPointer<vtkFloatArray>::New();
  intensity->SetName("intensity");
  intensity->SetNumberOfValues(numberOfPoints);
  polyData->GetPointData()->AddArray(intensity);


  sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_i(*msg, "intensity");

  for (size_t i = 0; i < numberOfPoints; ++i, ++iter_x, ++iter_i) {
    //printf("xyzi: %f %f %f %f\n", iter_x[0], iter_x[1], iter_x[2], iter_i[0]);
    points->InsertNextPoint(&iter_x[0]);
    intensity->SetValue(i, iter_i[0]);
  }

  return polyData;
}

/*
  /*
  ROS_INFO_STREAM("Received point cloud with " << msg->height*msg->width << " points.");
  std::cout << "Point step: " << msg->point_step << std::endl;
  std::cout << "Row step: " << msg->row_step << std::endl;
  std::cout << "Expected: " << msg->width*msg->point_step << std::endl;
  std::cout << "is_dense: " << bool(msg->is_dense) << std::endl;
  std::cout << "num fields " << msg->fields.size() << std::endl;
  for (size_t i = 0; i < msg->fields.size(); ++i) {
    std::cout << "  field: " << msg->fields[i].name << std::endl;
    std::cout << "   offset: " << int(msg->fields[i].offset) << std::endl;
    std::cout << "   dtype: " << int(msg->fields[i].datatype) << std::endl;
    std::cout << "   count: " << int(msg->fields[i].count) << std::endl;

  }

  size_t npoints = msg->height*msg->width;
  sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_i(*msg, "intensity");

  for (size_t i = 0; i <2; ++i, ++iter_x, ++iter_i) {
    printf("xyzi: %f %f %f %f\n", iter_x[0], iter_x[1], iter_x[2], iter_i[0]);
  }
*/

}


#endif
