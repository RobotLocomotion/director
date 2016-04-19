/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkSimpleActorInteractor.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkSimpleActorInteractor -

// .SECTION Description


#ifndef __vtkSimpleActorInteractor_h
#define __vtkSimpleActorInteractor_h

#include "vtkInteractorStyleTrackballActor.h"
#include "ddAppConfigure.h"


class DD_APP_EXPORT vtkSimpleActorInteractor : public vtkInteractorStyleTrackballActor
{
public:
  static vtkSimpleActorInteractor *New();
  vtkTypeMacro(vtkSimpleActorInteractor, vtkInteractorStyleTrackballActor);
  void PrintSelf(ostream& os, vtkIndent indent);

  // Description:
  // Override parent class methods to remap the interaction style
  virtual void OnLeftButtonDown();
  virtual void OnLeftButtonUp();
  virtual void OnMiddleButtonDown();
  virtual void OnMiddleButtonUp();
  virtual void OnRightButtonDown();
  virtual void OnRightButtonUp();

  // Description:
  // Override parent class method to dolly camera instead of actor
  virtual void Dolly();

protected:

  vtkSimpleActorInteractor();
  virtual ~vtkSimpleActorInteractor();


private:
  vtkSimpleActorInteractor(const vtkSimpleActorInteractor&);  // Not implemented.
  void operator=(const vtkSimpleActorInteractor&);  // Not implemented.
};
#endif
