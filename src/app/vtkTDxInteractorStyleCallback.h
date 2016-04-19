/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkTDxInteractorStyleCallback.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkTDxInteractorStyleCallback - sends TDx events to observers

// .SECTION Description
// vtkTDxInteractorStyleCallback sends motion and button events from a
// 3DConnexion device to observers.

#ifndef __vtkTDxInteractorStyleCallback_h
#define __vtkTDxInteractorStyleCallback_h

#include "vtkTDxInteractorStyle.h"
#include "ddAppConfigure.h"

class vtkTransform;

class DD_APP_EXPORT vtkTDxInteractorStyleCallback : public vtkTDxInteractorStyle
{
public:
  static vtkTDxInteractorStyleCallback *New();
  vtkTypeMacro(vtkTDxInteractorStyleCallback, vtkTDxInteractorStyle);
  void PrintSelf(ostream& os, vtkIndent indent);

  vtkGetVector3Macro(Translation, double);
  vtkGetVector4Macro(AngleAxis, double);

  virtual void OnMotionEvent(vtkTDxMotionEventInfo *motionInfo);

  virtual void OnButtonPressedEvent(int button);

  virtual void OnButtonReleasedEvent(int button);

protected:

  vtkTDxInteractorStyleCallback();
  virtual ~vtkTDxInteractorStyleCallback();

  int Button;
  double Translation[3];
  double AngleAxis[4];

private:
  vtkTDxInteractorStyleCallback(const vtkTDxInteractorStyleCallback&);  // Not implemented.
  void operator=(const vtkTDxInteractorStyleCallback&);  // Not implemented.
};
#endif
