/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkMapServerSource.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkMapServerSource -
// .SECTION Description
//

#ifndef __vtkMapServerSource_h
#define __vtkMapServerSource_h

#include <vtkPolyDataAlgorithm.h>

#include <vtkDRCFiltersModule.h>

class vtkImageData;
class vtkTransform;

class VTKDRCFILTERS_EXPORT vtkMapServerSource : public vtkPolyDataAlgorithm
{
public:
  vtkTypeMacro(vtkMapServerSource, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent) VTKDRCFILTERS_OVERRIDE;

  static vtkMapServerSource *New();


  void Poll();

  void Start();
  void Stop();

  vtkGetVector2Macro(DistanceRange, double);
  vtkSetVector2Macro(DistanceRange, double);

  vtkGetVector2Macro(HeightRange, double);
  vtkSetVector2Macro(HeightRange, double);

  void SetEdgeAngleThreshold(double threshold);
  double GetEdgeAngleThreshold();

  int GetNumberOfDatasets(int viewId);
  vtkPolyData* GetDataset(int viewId, vtkIdType i);

  vtkIdType GetCurrentMapId(int viewId);
  void GetDataForMapId(int viewId, vtkIdType mapId, vtkPolyData* polyData);
  void GetMeshForMapId(int viewId, vtkIdType mapId, vtkPolyData* polyData);
  void GetDataForMapId(int viewId, vtkIdType mapId, vtkImageData* imageData, vtkTransform* transform);
  vtkIdType GetLastScanBundleUTime();


  vtkIntArray* GetViewIds();

protected:


  virtual int RequestInformation(vtkInformation *request,
                         vtkInformationVector **inputVector,
                         vtkInformationVector *outputVector) VTKDRCFILTERS_OVERRIDE;

  virtual int RequestData(vtkInformation *request,
                          vtkInformationVector **inputVector,
                          vtkInformationVector *outputVector) VTKDRCFILTERS_OVERRIDE;

  vtkMapServerSource();
  virtual ~vtkMapServerSource() VTKDRCFILTERS_OVERRIDE;

  double DistanceRange[2];
  double EdgeAngleThreshold;
  double HeightRange[2];

private:
  vtkMapServerSource(const vtkMapServerSource&) VTKDRCFILTERS_DELETE_FUNCTION;
  void operator=(const vtkMapServerSource&) VTKDRCFILTERS_DELETE_FUNCTION;

  class vtkInternal;
  vtkInternal * Internal;
};

#endif
