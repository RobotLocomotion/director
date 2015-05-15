#ifndef __vtkPlaneSegmentation_h
#define __vtkPlaneSegmentation_h

#include <vtkPolyDataAlgorithm.h>
#include <vtkDRCFiltersModule.h>

class VTKDRCFILTERS_EXPORT vtkPlaneSegmentation : public vtkPolyDataAlgorithm
{
public:
  vtkTypeMacro(vtkPlaneSegmentation, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  static vtkPlaneSegmentation *New();

  vtkSetMacro(DistanceThreshold, double);
  vtkGetMacro(DistanceThreshold, double);

  vtkSetMacro(MaxIterations, int);
  vtkGetMacro(MaxIterations, int);

  vtkGetVector4Macro(PlaneCoefficients, double);
  vtkGetVector3Macro(PlaneOrigin, double);
  vtkGetVector3Macro(PlaneNormal, double);

  vtkSetMacro(PerpendicularConstraintEnabled, bool);
  vtkGetMacro(PerpendicularConstraintEnabled, bool);

  vtkSetMacro(AngleEpsilon, double);
  vtkGetMacro(AngleEpsilon, double);

  vtkGetVector3Macro(PerpendicularAxis, double);
  vtkSetVector3Macro(PerpendicularAxis, double);

protected:

  double DistanceThreshold;
  int MaxIterations;

  bool PerpendicularConstraintEnabled;
  double PerpendicularAxis[3];
  double AngleEpsilon;

  double PlaneCoefficients[4];
  double PlaneOrigin[3];
  double PlaneNormal[3];

  virtual int RequestData(vtkInformation *request,
                          vtkInformationVector **inputVector,
                          vtkInformationVector *outputVector);


  vtkPlaneSegmentation();
  virtual ~vtkPlaneSegmentation();

private:
  vtkPlaneSegmentation(const vtkPlaneSegmentation&);  // Not implemented.
  void operator=(const vtkPlaneSegmentation&);  // Not implemented.
};

#endif
