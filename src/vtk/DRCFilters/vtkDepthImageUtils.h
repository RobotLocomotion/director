// .NAME vtkDepthImageUtils - utilities functions for converting depth images
// .Section Description
//

#ifndef _vtkDepthImageUtils_h
#define _vtkDepthImageUtils_h

#include <vtkPolyDataAlgorithm.h>
#include <vtkSmartPointer.h>

#include <vtkDRCFiltersModule.h>

class vtkImageData;
class vtkCamera;
class vtkPoints;
class vtkUnsignedCharArray;

class VTKDRCFILTERS_EXPORT vtkDepthImageUtils : public vtkPolyDataAlgorithm
{
public:
  static vtkDepthImageUtils *New();
  vtkTypeMacro(vtkDepthImageUtils, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);


  static void DepthBufferToDepthImage(vtkImageData* depthBuffer, vtkImageData* colorBuffer,
                                      vtkCamera* camera, vtkImageData* depthImage,
                                      vtkPoints* pts, vtkUnsignedCharArray* ptColors);

protected:
  vtkDepthImageUtils();
  ~vtkDepthImageUtils();

private:

  vtkDepthImageUtils(const vtkDepthImageUtils&);
  void operator = (const vtkDepthImageUtils&);

};
#endif
