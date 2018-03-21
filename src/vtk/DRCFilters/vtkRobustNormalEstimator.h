#ifndef __vtkRobustNormalEstimator_h
#define __vtkRobustNormalEstimator_h

#include <vtkPolyDataAlgorithm.h>
#include <vtkDRCFiltersModule.h>


class VTKDRCFILTERS_EXPORT vtkRobustNormalEstimator : public vtkPolyDataAlgorithm
{

public:
  vtkTypeMacro(vtkRobustNormalEstimator, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent) VTKDRCFILTERS_OVERRIDE;

  static vtkRobustNormalEstimator *New();

  // Description:
  // Search radius for normal estimation
  vtkSetMacro(Radius, double);
  vtkGetMacro(Radius, double);

  // Description:
  // Distance error between a point being processed and the local plane fit.
  // If the distance is greater than MaxCenterError then the point will not be
  // assigned a valid normal.
  vtkSetMacro(MaxCenterError, double);
  vtkGetMacro(MaxCenterError, double);

  // Description:
  // Distance error threshold for ransac plane fitting.
  vtkSetMacro(MaxEstimationError, double);
  vtkGetMacro(MaxEstimationError, double);

  // Description:
  // Iterations limit for ransac plane fitting.
  vtkSetMacro(MaxIterations, int);
  vtkGetMacro(MaxIterations, int);

  vtkSetMacro(ComputeCurvature, bool);
  vtkGetMacro(ComputeCurvature, bool);

protected:

  bool ComputeCurvature;
  int MaxIterations;
  double Radius;
  double MaxEstimationError;
  double MaxCenterError;

  virtual int RequestData(vtkInformation *request,
                          vtkInformationVector **inputVector,
                          vtkInformationVector *outputVector) VTKDRCFILTERS_OVERRIDE;


  vtkRobustNormalEstimator();
  virtual ~vtkRobustNormalEstimator() VTKDRCFILTERS_OVERRIDE;

private:
  vtkRobustNormalEstimator(const vtkRobustNormalEstimator&)
    VTKDRCFILTERS_DELETE_FUNCTION;
  void operator=(const vtkRobustNormalEstimator&)
    VTKDRCFILTERS_DELETE_FUNCTION;
};

#endif
