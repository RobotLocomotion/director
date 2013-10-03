/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkSimpleActorInteractor.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#include "vtkSimpleActorInteractor.h"

#include <vtkObjectFactory.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkCallbackCommand.h>
#include <vtkCamera.h>
#include <vtkRenderer.h>

vtkStandardNewMacro(vtkSimpleActorInteractor);

// ----------------------------------------------------------------------------
vtkSimpleActorInteractor::vtkSimpleActorInteractor()
{

}

// ----------------------------------------------------------------------------
vtkSimpleActorInteractor::~vtkSimpleActorInteractor()
{

}

//----------------------------------------------------------------------------
void vtkSimpleActorInteractor::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
}


//----------------------------------------------------------------------------
void vtkSimpleActorInteractor::OnLeftButtonDown()
{
  int x = this->Interactor->GetEventPosition()[0];
  int y = this->Interactor->GetEventPosition()[1];

  this->FindPokedRenderer(x, y);
  this->FindPickedActor(x, y);
  if (this->CurrentRenderer == NULL || this->InteractionProp == NULL)
    {
    return;
    }

  this->GrabFocus(this->EventCallbackCommand);

  this->StartPan();
}

//----------------------------------------------------------------------------
void vtkSimpleActorInteractor::OnLeftButtonUp()
{
  this->EndPan();

  if (this->Interactor)
    {
    this->ReleaseFocus();
    }
}

//----------------------------------------------------------------------------
void vtkSimpleActorInteractor::OnMiddleButtonDown()
{
  int x = this->Interactor->GetEventPosition()[0];
  int y = this->Interactor->GetEventPosition()[1];

  this->FindPokedRenderer(x, y);
  if (this->CurrentRenderer == NULL)
    {
    return;
    }

  this->GrabFocus(this->EventCallbackCommand);

  this->StartDolly();
}

//----------------------------------------------------------------------------
void vtkSimpleActorInteractor::OnMiddleButtonUp()
{
  this->EndDolly();

  if (this->Interactor)
    {
    this->ReleaseFocus();
    }
}

//----------------------------------------------------------------------------
void vtkSimpleActorInteractor::OnRightButtonDown()
{
  int x = this->Interactor->GetEventPosition()[0];
  int y = this->Interactor->GetEventPosition()[1];

  this->FindPokedRenderer(x, y);
  this->FindPickedActor(x, y);
  if (this->CurrentRenderer == NULL || this->InteractionProp == NULL)
    {
    return;
    }

  this->GrabFocus(this->EventCallbackCommand);

  this->StartSpin();
}

//----------------------------------------------------------------------------
void vtkSimpleActorInteractor::OnRightButtonUp()
{
  this->EndSpin();

  if ( this->Interactor )
    {
    this->ReleaseFocus();
    }
}

//----------------------------------------------------------------------------
void vtkSimpleActorInteractor::Dolly()
{
  if (this->CurrentRenderer == NULL)
    {
    return;
    }

  vtkRenderWindowInteractor *rwi = this->Interactor;
  vtkCamera *cam = this->CurrentRenderer->GetActiveCamera();

  double view_point[3], view_focus[3];
  double motion_vector[3];

  cam->GetPosition(view_point);
  cam->GetFocalPoint(view_focus);

  double *center = this->CurrentRenderer->GetCenter();

  int dy = rwi->GetEventPosition()[1] - rwi->GetLastEventPosition()[1];

  double yf = dy / center[1] * this->MotionFactor;
  double dollyFactor = pow(1.1, yf);

  dollyFactor -= 1.0;
  dollyFactor *= -1;

  if (cam->GetParallelProjection())
    {
    cam->SetParallelScale(cam->GetParallelScale()*(1+dollyFactor));
    }
  else
    {
    cam->Dolly(dollyFactor);
    }

  if (this->AutoAdjustCameraClippingRange)
    {
    this->CurrentRenderer->ResetCameraClippingRange();
    }

  rwi->Render();
}
