#include "vtkDepthImageUtils.h"

#include "vtkNew.h"
#include "vtkSmartPointer.h"
#include "vtkPolyData.h"
#include "vtkMath.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"
#include "vtkTransform.h"
#include "vtkMatrix4x4.h"
#include "vtkCamera.h"
#include "vtkImageData.h"
#include "vtkUnsignedCharArray.h"
#include <Eigen/Dense>

//-----------------------------------------------------------------------------
vtkStandardNewMacro(vtkDepthImageUtils);

//-----------------------------------------------------------------------------
vtkDepthImageUtils::vtkDepthImageUtils()
{

}

//-----------------------------------------------------------------------------
vtkDepthImageUtils::~vtkDepthImageUtils()
{
}

namespace {

  Eigen::Matrix4f toEigenMatrix(vtkMatrix4x4* mat)
  {
    Eigen::Matrix4f eigenMat;

    for (int i = 0; i < 4; ++i)
    {
      for (int j = 0; j < 4; ++j)
      {
        eigenMat(i, j) = static_cast<float>(mat->Element[i][j]);
      }
    }
    return eigenMat;
  }
}

//-----------------------------------------------------------------------------
void vtkDepthImageUtils::DepthBufferToDepthImage(vtkImageData* depthBuffer, vtkImageData* colorBuffer, vtkCamera* camera, vtkImageData* depthImage, vtkPoints* pts, vtkUnsignedCharArray* ptColors)
{
  depthImage->DeepCopy(depthBuffer);
  ptColors->SetNumberOfComponents(3);

  int imageWidth = depthImage->GetDimensions()[0];
  int imageHeight = depthImage->GetDimensions()[1];
  double aspectRatio = static_cast<double>(imageWidth)/imageHeight;

  float* depthData = static_cast<float*>(depthImage->GetScalarPointer(0, 0, 0));
  unsigned char* colorData = static_cast<unsigned char*>(colorBuffer->GetScalarPointer(0, 0, 0));

  Eigen::Matrix4f cameraToViewport = toEigenMatrix(camera->GetProjectionTransformMatrix(aspectRatio, 0, 1));
  Eigen::Matrix4f viewportToCamera = cameraToViewport.inverse();
  Eigen::Matrix4f worldToCamera = toEigenMatrix(camera->GetViewTransformMatrix());
  Eigen::Matrix4f cameraToWorld = worldToCamera.inverse();

  int ptr = 0;
  int colorPtr = 0;
  for (int y = 0; y < imageHeight; ++y)
  {
    for (int x = 0; x < imageWidth; ++x, ++ptr, colorPtr+=3)
    {
      float z = depthData[ptr];
      if (z == 1.0)
      {
        depthData[ptr] = std::numeric_limits<float>::quiet_NaN();
        continue;
      }

      Eigen::Vector4f ptToViewport(2*float(x)/imageWidth - 1,
                                  2*float(y)/imageHeight - 1,
                                  z,
                                  1.0);


      Eigen::Vector4f ptToCamera = viewportToCamera * ptToViewport;

      float w3 = 1.0f / ptToCamera[3];

      ptToCamera[0] *= w3;
      ptToCamera[1] *= w3;
      ptToCamera[2] *= w3;
      ptToCamera[3] = 1.0;

      Eigen::Vector4f ptToWorld = cameraToWorld * ptToCamera;
      depthData[ptr] = -ptToCamera[2];

      //pts->InsertNextPoint(ptToWorld[0], ptToWorld[1], ptToWorld[2]);
      pts->InsertNextPoint(ptToCamera[0], ptToCamera[1], ptToCamera[2]);
      ptColors->InsertNextTupleValue(&colorData[colorPtr]);
    }
  }
}

//-----------------------------------------------------------------------------
void vtkDepthImageUtils::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}
