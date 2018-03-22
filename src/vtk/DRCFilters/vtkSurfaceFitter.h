#ifndef __vtkSurfaceFitter_h
#define __vtkSurfaceFitter_h

#include <vtkPolyDataAlgorithm.h>
#include <vtkDRCFiltersModule.h>

class vtkPlane;
class vtkRectd;
class vtkRobustNormalEstimator;

class VTKDRCFILTERS_EXPORT vtkSurfaceFitter : public vtkPolyDataAlgorithm
{
public:
  vtkTypeMacro(vtkSurfaceFitter, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent) VTKDRCFILTERS_OVERRIDE;

  static vtkSurfaceFitter *New();

  // Max average distance error from points to the plane estimate.
  // Region growing will not add a point that increases the average
  // error beyond this threshold.
  vtkSetMacro(MaxError, double);
  vtkGetMacro(MaxError, double);

  // Max angle error between a point normal and the estimated plane normal.
  // Region growing will not add a point whose local normal is more than
  // the max angle different from the estimated plane normal.
  vtkSetMacro(MaxAngle, double);
  vtkGetMacro(MaxAngle, double);

  // This controls the search distance for region growing from point to point.
  vtkSetMacro(SearchRadius, double);
  vtkGetMacro(SearchRadius, double);

  // Regions below this number of points will be dropped from the results list.
  vtkSetMacro(MinimumNumberOfPoints, int);
  vtkGetMacro(MinimumNumberOfPoints, int);


  // Description:
  // Access this object in order to set the normal estimation parameters
  // of the robust normal estimator.
  vtkGetObjectMacro(RobustNormalEstimator, vtkRobustNormalEstimator);


  static void ComputePlane(vtkPolyData* polyData, vtkPlane* plane);
  static void ComputeConvexHull(vtkPolyData* polyData, vtkPlane* plane, vtkPolyData* convexHull);
  static void ComputeMinimumAreaRectangleFit(vtkPolyData* polyData, vtkPlane* plane, vtkRectd* rectangle);
  static void ComputeKnownSizeRectangleFit(vtkPolyData* polyData, vtkPlane* plane, vtkRectd* rectangle);


protected:

  double MaxError;
  double MaxAngle;
  double SearchRadius;
  int MinimumNumberOfPoints;
  vtkRobustNormalEstimator* RobustNormalEstimator;

  virtual int RequestData(vtkInformation *request,
                          vtkInformationVector **inputVector,
                          vtkInformationVector *outputVector) VTKDRCFILTERS_OVERRIDE;


  vtkSurfaceFitter();
  virtual ~vtkSurfaceFitter() VTKDRCFILTERS_OVERRIDE;

private:
  vtkSurfaceFitter(const vtkSurfaceFitter&) VTKDRCFILTERS_DELETE_FUNCTION;
  void operator=(const vtkSurfaceFitter&) VTKDRCFILTERS_DELETE_FUNCTION;
};

#endif
