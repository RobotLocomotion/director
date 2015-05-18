#include "vtkPlaneSegmentation.h"
//#include "vtkPCLConversions.h"

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

#include <plane-seg/PlaneFitter.hpp>


//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkPlaneSegmentation);

//----------------------------------------------------------------------------
vtkPlaneSegmentation::vtkPlaneSegmentation()
{
  this->DistanceThreshold = 0.05;
  this->MaxIterations = 200;

  this->PlaneCoefficients[0] = 0.0;
  this->PlaneCoefficients[1] = 0.0;
  this->PlaneCoefficients[2] = 0.0;
  this->PlaneCoefficients[3] = 0.0;

  this->PerpendicularConstraintEnabled = false;
  this->AngleEpsilon = 0.2;
  this->PerpendicularAxis[0] = 1.0;
  this->PerpendicularAxis[1] = 0.0;
  this->PerpendicularAxis[2] = 0.0;

  this->SetNumberOfInputPorts(1);
  this->SetNumberOfOutputPorts(1);
}

//----------------------------------------------------------------------------
vtkPlaneSegmentation::~vtkPlaneSegmentation()
{
}

//----------------------------------------------------------------------------
int vtkPlaneSegmentation::RequestData(
  vtkInformation* vtkNotUsed(request),
  vtkInformationVector **inputVector,
  vtkInformationVector *outputVector)
{
  // get input and output data objects
  vtkInformation *inInfo = inputVector[0]->GetInformationObject(0);
  vtkPolyData *input = vtkPolyData::SafeDownCast(inInfo->Get(vtkDataObject::DATA_OBJECT()));
  vtkInformation *outInfo = outputVector->GetInformationObject(0);
  vtkPolyData *output = vtkPolyData::SafeDownCast(outInfo->Get(vtkDataObject::DATA_OBJECT()));

  // convert point cloud
  std::vector<Eigen::Vector3f> pointList;
  const vtkIdType numberOfPoints = input->GetNumberOfPoints();
  pointList.resize(numberOfPoints);
  vtkFloatArray* floatPoints = vtkFloatArray::SafeDownCast(input->GetPoints()->GetData());
  vtkDoubleArray* doublePoints = vtkDoubleArray::SafeDownCast(input->GetPoints()->GetData());
  assert((floatPoints != NULL) || (doublePoints != NULL));
  if (floatPoints != NULL) {
    float* data = floatPoints->GetPointer(0);
    for (vtkIdType i = 0; i < numberOfPoints; ++i) {
      pointList[i] << data[i*3], data[i*3+1], data[i*3+2];
    }
  }
  else if (doublePoints != NULL) {
    double* data = doublePoints->GetPointer(0);
    for (vtkIdType i = 0; i < numberOfPoints; ++i) {
      pointList[i] << data[i*3], data[i*3+1], data[i*3+2];
    }
  }

  // perform plane model fit
  planeseg::PlaneFitter fitter;
  fitter.setMaxDistance(this->DistanceThreshold);
  fitter.setMaxIterations(this->MaxIterations);
  fitter.setRefineUsingInliers(true);
  if (this->PerpendicularConstraintEnabled) {
    Eigen::Vector3f prior(this->PerpendicularAxis[0],
                          this->PerpendicularAxis[1],
                          this->PerpendicularAxis[2]);
    fitter.setNormalPrior(prior, this->AngleEpsilon);
  }
  planeseg::PlaneFitter::Result result = fitter.go(pointList);
  if (!result.mSuccess) {
    vtkErrorMacro("Error segmenting plane.");
    return 0;
  }

  // store plane coefficients
  if (result.mPlane[2] < 0) result.mPlane = -result.mPlane;
  for (size_t i = 0; i < 4; ++i) {
    this->PlaneCoefficients[i] = result.mPlane[i];
  }
  vtkNew<vtkPlane> plane;
  Eigen::Vector3d normal = result.mPlane.head<3>().cast<double>();
  plane->SetNormal(normal.data());
  plane->Push(-result.mPlane[3]);
  plane->GetOrigin(this->PlaneOrigin);
  plane->GetNormal(this->PlaneNormal);

  // pass thru input add labels
  vtkSmartPointer<vtkIntArray> labels = vtkSmartPointer<vtkIntArray>::New();
  labels->SetNumberOfComponents(1);
  labels->SetNumberOfTuples(pointList.size());
  labels->FillComponent(0, 0);
  for (size_t k = 0; k < result.mInliers.size(); ++k) {
    labels->SetValue(result.mInliers[k], 1);
  }
  labels->SetName("ransac_labels");
  output->ShallowCopy(input);
  output->GetPointData()->AddArray(labels);

  return 1;
}

//----------------------------------------------------------------------------
void vtkPlaneSegmentation::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
}
