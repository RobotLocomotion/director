/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkLidarSource.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkLidarSource -
// .SECTION Description
//

#ifndef __vtkLidarSource_h
#define __vtkLidarSource_h

#include <vtkPolyDataAlgorithm.h>

#include <vtkDRCFiltersModule.h>

class vtkTransform;

class VTKDRCFILTERS_EXPORT vtkLidarSource : public vtkPolyDataAlgorithm
{
public:
  vtkTypeMacro(vtkLidarSource, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  static vtkLidarSource *New();


  void Poll();

  void Start();
  void Stop();

  vtkGetVector2Macro(DistanceRange, double);
  vtkSetVector2Macro(DistanceRange, double);

  vtkGetVector2Macro(HeightRange, double);
  vtkSetVector2Macro(HeightRange, double);

  void SetEdgeAngleThreshold(double threshold);
  double GetEdgeAngleThreshold();

  int GetCurrentRevolution();
  void GetDataForRevolution(int revolution, vtkPolyData* polyData);

  void GetDataForHistory(int numberOfScanLines, vtkPolyData* polyData);

  int GetCurrentScanLine();
  void GetDataForScanLine(int scanLine, vtkPolyData* polyData);
  vtkIdType GetCurrentScanTime();

  void InitBotConfig(const char* filename);

  void GetTransform(const char* fromFrame, const char* toFrame, vtkIdType utime, vtkTransform* transform);

  static void GetBotRollPitchYaw(vtkTransform* transform, double rpy[3]);
  static void GetBotQuaternion(vtkTransform* transform, double wxyz[4]);

protected:


  virtual int RequestInformation(vtkInformation *request,
                         vtkInformationVector **inputVector,
                         vtkInformationVector *outputVector);

  virtual int RequestData(vtkInformation *request,
                          vtkInformationVector **inputVector,
                          vtkInformationVector *outputVector);

  vtkLidarSource();
  virtual ~vtkLidarSource();

  double DistanceRange[2];
  double EdgeAngleThreshold;
  double HeightRange[2];

private:
  vtkLidarSource(const vtkLidarSource&);  // Not implemented.
  void operator=(const vtkLidarSource&);  // Not implemented.

  class vtkInternal;
  vtkInternal * Internal;
};

#endif
