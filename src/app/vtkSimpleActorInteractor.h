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
  void PrintSelf(ostream& os, vtkIndent indent) DD_APP_OVERRIDE;

  // Description:
  // Override parent class methods to remap the interaction style
  virtual void OnLeftButtonDown() DD_APP_OVERRIDE;
  virtual void OnLeftButtonUp() DD_APP_OVERRIDE;
  virtual void OnMiddleButtonDown() DD_APP_OVERRIDE;
  virtual void OnMiddleButtonUp() DD_APP_OVERRIDE;
  virtual void OnRightButtonDown() DD_APP_OVERRIDE;
  virtual void OnRightButtonUp() DD_APP_OVERRIDE;

  // Description:
  // Override parent class method to dolly camera instead of actor
  virtual void Dolly() DD_APP_OVERRIDE;

protected:

  vtkSimpleActorInteractor();
  virtual ~vtkSimpleActorInteractor() DD_APP_OVERRIDE;


private:
  vtkSimpleActorInteractor(const vtkSimpleActorInteractor&)
    DD_APP_DELETE_FUNCTION;
  void operator=(const vtkSimpleActorInteractor&) DD_APP_DELETE_FUNCTION;
};
#endif
