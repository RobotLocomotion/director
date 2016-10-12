#include "vtkRobustNormalEstimator.h"

#include "vtkPolyData.h"
#include "vtkPointData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"
#include "vtkSmartPointer.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkPlane.h"
#include "vtkNew.h"
#include "vtkFloatArray.h"
#include "vtkDoubleArray.h"

#include <pcl/common/io.h>
#include <plane-seg/RobustNormalEstimator.hpp>


//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkRobustNormalEstimator);

//----------------------------------------------------------------------------
vtkRobustNormalEstimator::vtkRobustNormalEstimator()
{
  this->Radius = 0.1;
  this->MaxCenterError = 0.02;
  this->MaxEstimationError = 0.01;
  this->MaxIterations = 100;
  this->ComputeCurvature = false;

  this->SetNumberOfInputPorts(1);
  this->SetNumberOfOutputPorts(1);
}

//----------------------------------------------------------------------------
vtkRobustNormalEstimator::~vtkRobustNormalEstimator()
{
}


namespace {


//----------------------------------------------------------------------------
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudFromPolyData(vtkPolyData* polyData)
{
  const vtkIdType numberOfPoints = polyData->GetNumberOfPoints();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->width = numberOfPoints;
  cloud->height = 1;
  cloud->is_dense = true;
  cloud->points.resize(numberOfPoints);

  if (!numberOfPoints)
    {
    return cloud;
    }

  vtkFloatArray* floatPoints = vtkFloatArray::SafeDownCast(polyData->GetPoints()->GetData());
  vtkDoubleArray* doublePoints = vtkDoubleArray::SafeDownCast(polyData->GetPoints()->GetData());
  assert(floatPoints || doublePoints);

  if (floatPoints)
    {
    float* data = floatPoints->GetPointer(0);
    for (vtkIdType i = 0; i < numberOfPoints; ++i)
      {
      cloud->points[i].x = data[i*3];
      cloud->points[i].y = data[i*3+1];
      cloud->points[i].z = data[i*3+2];
      }
    }
  else if (doublePoints)
    {
    double* data = doublePoints->GetPointer(0);
    for (vtkIdType i = 0; i < numberOfPoints; ++i)
      {
      cloud->points[i].x = data[i*3];
      cloud->points[i].y = data[i*3+1];
      cloud->points[i].z = data[i*3+2];
      }
    }

  return cloud;
}

}

//----------------------------------------------------------------------------
int vtkRobustNormalEstimator::RequestData(
  vtkInformation* vtkNotUsed(request),
  vtkInformationVector **inputVector,
  vtkInformationVector *outputVector)
{
  // get input and output data objects
  vtkInformation *inInfo = inputVector[0]->GetInformationObject(0);
  vtkPolyData *input = vtkPolyData::SafeDownCast(inInfo->Get(vtkDataObject::DATA_OBJECT()));

  vtkInformation *outInfo = outputVector->GetInformationObject(0);
  vtkPolyData *output = vtkPolyData::SafeDownCast(outInfo->Get(vtkDataObject::DATA_OBJECT()));


  // create new normals array
  vtkSmartPointer<vtkFloatArray> normals = vtkSmartPointer<vtkFloatArray>::New();
  normals->SetNumberOfComponents(3);
  normals->SetNumberOfTuples(input->GetNumberOfPoints());
  normals->SetName("normals");

  // pass input thru to output and add new array
  output->ShallowCopy(input);
  output->GetPointData()->AddArray(normals);
  output->GetPointData()->SetNormals(normals);

  // early exit if input data has no points
  if (!input->GetNumberOfPoints())
    {
    return 1;
    }

  // convert point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = PointCloudFromPolyData(input);

  planeseg::LabeledCloud::Ptr lcloud(new planeseg::LabeledCloud);

  pcl::copyPointCloud<pcl::PointXYZ, planeseg::Point>(*cloud, *lcloud);

  planeseg::RobustNormalEstimator normalEstimator;
  normalEstimator.setMaxEstimationError(this->GetMaxEstimationError());
  normalEstimator.setRadius(this->GetRadius());
  normalEstimator.setMaxCenterError(this->GetMaxCenterError());
  normalEstimator.setMaxIterations(this->GetMaxIterations());
  normalEstimator.computeCurvature(this->GetComputeCurvature());
  planeseg::NormalCloud cloudNormals;
  normalEstimator.go(lcloud, cloudNormals);


  assert(cloudNormals.size() == normals->GetNumberOfTuples());

  for (size_t i = 0; i < cloudNormals.size(); ++i)
    {
    normals->SetTuple(i, cloudNormals.points[i].normal);
    }

  if (this->GetComputeCurvature())
    {
    vtkSmartPointer<vtkDoubleArray> curvature = vtkSmartPointer<vtkDoubleArray>::New();
    curvature->SetNumberOfComponents(1);
    curvature->SetNumberOfTuples(cloudNormals.size());

    for (size_t i = 0; i < cloudNormals.size(); ++i)
      {
      curvature->SetValue(i, cloudNormals.points[i].curvature);
      }

    curvature->SetName("curvature");
    output->GetPointData()->AddArray(curvature);
    }

  return 1;
}

//----------------------------------------------------------------------------
void vtkRobustNormalEstimator::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
}
