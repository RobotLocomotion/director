// Copyright 2013 Velodyne Acoustics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkGridSource.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkGridSource - generates a vtkPolyData measurement grid plane
// .Section Description
//

#ifndef _vtkGridSource_h
#define _vtkGridSource_h

#include <vtkPolyDataAlgorithm.h>
#include <vtkSmartPointer.h>

#include <vtkDRCFiltersModule.h>

class VTKDRCFILTERS_EXPORT vtkGridSource : public vtkPolyDataAlgorithm
{
public:
  static vtkGridSource *New();
  vtkTypeMacro(vtkGridSource, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  vtkSetMacro(GridSize, int);
  vtkGetMacro(GridSize, int);

  vtkSetMacro(Scale, double);
  vtkGetMacro(Scale, double);

  vtkSetVector3Macro(Origin, double);
  vtkGetVector3Macro(Origin, double);

  vtkSetVector3Macro(Normal, double);
  vtkGetVector3Macro(Normal, double);

  vtkSetMacro(ArcsEnabled, bool);
  vtkGetMacro(ArcsEnabled, bool);
  vtkBooleanMacro(ArcsEnabled, bool);

  vtkSetMacro(SurfaceEnabled, bool);
  vtkGetMacro(SurfaceEnabled, bool);
  vtkBooleanMacro(SurfaceEnabled, bool);

  static vtkSmartPointer<vtkPolyData> CreateGrid(int gridSize, double scale, double origin[3], double normal[3], bool useCircles, bool useSurface);

protected:
  vtkGridSource();
  ~vtkGridSource();

  int RequestData(vtkInformation *,
                  vtkInformationVector **,
                  vtkInformationVector *);

  bool ArcsEnabled;
  bool SurfaceEnabled;
  int GridSize;
  double Scale;
  double Origin[3];
  double Normal[3];

private:

  vtkGridSource(const vtkGridSource&);
  void operator = (const vtkGridSource&);

};
#endif
