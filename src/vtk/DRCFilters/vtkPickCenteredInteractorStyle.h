/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkPickCenteredInteractorStyle.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkPickCenteredInteractorStyle - manipulate camera in scene with natural view up (e.g., terrain)
// .SECTION Description
// vtkPickCenteredInteractorStyle is based on the vtkInteractorStyleTerrain
// but has been modified to orbit and dolly about a custom pick point.

// .SECTION See Also
// vtkInteractorObserver vtkInteractorStyle vtk3DWidget

#ifndef __vtkPickCenteredInteractorStyle_h
#define __vtkPickCenteredInteractorStyle_h

#include "vtkInteractorStyle.h"

#include <vtkDRCFiltersModule.h>


class VTKDRCFILTERS_EXPORT vtkPickCenteredInteractorStyle : public vtkInteractorStyle
{
public:

  static vtkPickCenteredInteractorStyle *New();

  vtkTypeMacro(vtkPickCenteredInteractorStyle,vtkInteractorStyle);
  void PrintSelf(ostream& os, vtkIndent indent);

  // Description:
  // Event bindings controlling the effects of pressing mouse buttons
  // or moving the mouse.
  virtual void OnMouseMove();
  virtual void OnLeftButtonDown();
  virtual void OnLeftButtonUp();
  virtual void OnMiddleButtonDown();
  virtual void OnMiddleButtonUp();
  virtual void OnRightButtonDown();
  virtual void OnRightButtonUp();
  virtual void OnChar();

  // These methods for the different interactions in different modes
  // are overridden in subclasses to perform the correct motion.
  virtual void Rotate();
  virtual void Pan();
  virtual void Dolly();

  vtkGetMacro(MotionFactor, double);
  vtkSetMacro(MotionFactor, double);

  vtkSetVector3Macro(CustomCenterOfRotation, double);
  vtkGetVector3Macro(CustomCenterOfRotation, double);

  double ComputeScale(const double position[3], vtkRenderer *renderer);

protected:
  vtkPickCenteredInteractorStyle();
  ~vtkPickCenteredInteractorStyle();

  double MotionFactor;
  double CustomCenterOfRotation[3];

private:
  vtkPickCenteredInteractorStyle(const vtkPickCenteredInteractorStyle&);  // Not implemented.
  void operator=(const vtkPickCenteredInteractorStyle&);  // Not implemented.
  
};

#endif

