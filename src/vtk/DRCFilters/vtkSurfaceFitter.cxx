#include "vtkSurfaceFitter.h"
#include "vtkRobustNormalEstimator.h"

#include "vtkPolyData.h"
#include "vtkPointData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"
#include "vtkSmartPointer.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkPlane.h"
#include "vtkPolygon.h"
#include "vtkNew.h"
#include "vtkFloatArray.h"
#include "vtkDoubleArray.h"

#include <pcl/common/io.h>
#include <pcl/surface/convex_hull.h>
#include <plane-seg/RobustNormalEstimator.hpp>
#include <plane-seg/PlaneSegmenter.hpp>


//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkSurfaceFitter);

//----------------------------------------------------------------------------
vtkSurfaceFitter::vtkSurfaceFitter()
{

  this->MaxError = 0.05;
  this->MaxAngle = 5.0*M_PI/180;
  this->SearchRadius = 0.03;
  this->MinimumNumberOfPoints = 100;
  this->RobustNormalEstimator = vtkRobustNormalEstimator::New();

  this->SetNumberOfInputPorts(1);
  this->SetNumberOfOutputPorts(1);
}

//----------------------------------------------------------------------------
vtkSurfaceFitter::~vtkSurfaceFitter()
{
  this->RobustNormalEstimator->Delete();
  this->RobustNormalEstimator = NULL;
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

Eigen::MatrixX3d PolyDataToEigen(vtkPolyData* polyData)
{

  const vtkIdType nPoints = polyData->GetNumberOfPoints();
  Eigen::MatrixX3d data(nPoints, 3);
  for (vtkIdType i = 0; i <nPoints; ++i)
    {
    Eigen::Vector3d pt;
    polyData->GetPoint(i, pt.data());
    data.row(i) = pt;
    }

  return data;
}

}

//----------------------------------------------------------------------------
void vtkSurfaceFitter::ComputePlane(vtkPolyData* polyData, vtkPlane* plane)
{
  Eigen::MatrixX3d data = PolyDataToEigen(polyData);
  Eigen::Vector3d avg = data.colwise().mean();
  data.rowwise() -= avg.transpose();
  auto svd = data.jacobiSvd(Eigen::ComputeFullV);

  Eigen::Vector3d normal = svd.matrixV().col(2);
  Eigen::Vector3d origin = avg;

  plane->SetOrigin(origin.data());
  plane->SetNormal(normal.data());
}

//----------------------------------------------------------------------------
void vtkSurfaceFitter::ComputeConvexHull(vtkPolyData* polyData, vtkPlane* plane, vtkPolyData* convexHull)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = PointCloudFromPolyData(polyData);

  double* origin = plane->GetOrigin();
  double* normal = plane->GetNormal();

  for (size_t i = 0; i < cloud->size(); ++i)
    {
    Eigen::Vector3d pt(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
    vtkPlane::ProjectPoint(pt.data(), origin, normal, pt.data());

    cloud->points[i].x = pt[0];
    cloud->points[i].y = pt[1];
    cloud->points[i].z = pt[2];
    }

  pcl::ConvexHull<pcl::PointXYZ> filter;
  pcl::PointCloud<pcl::PointXYZ> hull;
  filter.setInputCloud(cloud);
  filter.reconstruct(hull);

  size_t hullSize = hull.size();

  //printf("got %d hull points\n", hullSize);

  vtkSmartPointer<vtkPoints> p = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkPolygon> pp = vtkSmartPointer<vtkPolygon>::New();

  p->SetNumberOfPoints(hullSize);
  pp->GetPointIds()->SetNumberOfIds(hullSize);


  for (size_t i = 0; i < hull.size(); ++i)
    {
    p->InsertPoint(i, hull[i].x, hull[i].y, hull[i].z);
    pp->GetPointIds()->SetId(i, i);
    }

  convexHull->SetPoints(p);
  convexHull->Allocate(1,1);
  convexHull->InsertNextCell(pp->GetCellType(), pp->GetPointIds());
}

//----------------------------------------------------------------------------
void vtkSurfaceFitter::ComputeMinimumAreaRectangleFit(vtkPolyData* polyData, vtkPlane* plane, vtkRectd* rectangle)
{

}

//----------------------------------------------------------------------------
void vtkSurfaceFitter::ComputeKnownSizeRectangleFit(vtkPolyData* polyData, vtkPlane* plane, vtkRectd* rectangle)
{

}


//----------------------------------------------------------------------------
int vtkSurfaceFitter::RequestData(
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
  normalEstimator.setRadius(this->RobustNormalEstimator->GetRadius());
  normalEstimator.setMaxCenterError(this->RobustNormalEstimator->GetMaxCenterError());
  normalEstimator.setMaxEstimationError(this->RobustNormalEstimator->GetMaxEstimationError());
  normalEstimator.setMaxIterations(this->RobustNormalEstimator->GetMaxIterations());

  planeseg::NormalCloud::Ptr cloudNormals(new planeseg::NormalCloud);

  //printf("computing normals...\n");
  normalEstimator.go(lcloud, *cloudNormals);
  //printf("done.\n");

  assert(cloudNormals->size() == normals->GetNumberOfTuples());

  for (size_t i = 0; i < cloudNormals->size(); ++i)
    {
    normals->SetTuple(i, cloudNormals->points[i].normal);
    }


  //printf("segmenting planes...\n");

  // Segment planes
  planeseg::PlaneSegmenter segmenter;
  segmenter.setData(lcloud, cloudNormals);
  segmenter.setMaxError(this->MaxError);
  segmenter.setMaxAngle(this->MaxAngle * 180.0/M_PI);
  segmenter.setSearchRadius(this->SearchRadius);
  segmenter.setMinPoints(this->MinimumNumberOfPoints);
  planeseg::PlaneSegmenter::Result result = segmenter.go();

  // pass thru input add labels
  vtkSmartPointer<vtkIntArray> labels = vtkSmartPointer<vtkIntArray>::New();
  labels->SetNumberOfComponents(1);
  labels->SetNumberOfTuples(result.mLabels.size());
  labels->FillComponent(0, 0);
  for (size_t k = 0; k < result.mLabels.size(); ++k) {
    labels->SetValue(k, result.mLabels[k]);
  }
  labels->SetName("plane_segmentation_labels");
  output->GetPointData()->AddArray(labels);

  return 1;
}

//----------------------------------------------------------------------------
void vtkSurfaceFitter::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
}
