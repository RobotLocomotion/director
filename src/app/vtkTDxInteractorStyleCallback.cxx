/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkTDxInteractorStyleCallback.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#include "vtkTDxInteractorStyleCallback.h"

#include "vtkCommand.h"
#include "vtkTDxMotionEventInfo.h"
#include "vtkObjectFactory.h"
#include "vtkTDxInteractorStyleSettings.h"

vtkStandardNewMacro(vtkTDxInteractorStyleCallback);

// ----------------------------------------------------------------------------
vtkTDxInteractorStyleCallback::vtkTDxInteractorStyleCallback()
{
  this->Translation[0] = 0.0;
  this->Translation[0] = 0.0;
  this->Translation[0] = 0.0;

  this->AngleAxis[0] = 0.0;
  this->AngleAxis[0] = 0.0;
  this->AngleAxis[0] = 0.0;
  this->AngleAxis[0] = 0.0;

  this->Button = 0;
}

// ----------------------------------------------------------------------------
vtkTDxInteractorStyleCallback::~vtkTDxInteractorStyleCallback()
{

}

// ----------------------------------------------------------------------------
void vtkTDxInteractorStyleCallback::OnMotionEvent(vtkTDxMotionEventInfo *motionInfo)
{
  this->Translation[0] = motionInfo->X;
  this->Translation[1] = motionInfo->Y;
  this->Translation[2] = motionInfo->Z;

  this->AngleAxis[0] = motionInfo->Angle;
  this->AngleAxis[1] = motionInfo->AxisX;
  this->AngleAxis[2] = motionInfo->AxisY;
  this->AngleAxis[3] = motionInfo->AxisZ;

  this->InvokeEvent(vtkCommand::TDxMotionEvent);
}

// ----------------------------------------------------------------------------
void vtkTDxInteractorStyleCallback::OnButtonPressedEvent(int button)
{
  this->Button = button;
  this->InvokeEvent(vtkCommand::TDxButtonPressEvent);
}

// ----------------------------------------------------------------------------
void vtkTDxInteractorStyleCallback::OnButtonReleasedEvent(int button)
{
  this->Button = button;
  this->InvokeEvent(vtkCommand::TDxButtonReleaseEvent);
}

//----------------------------------------------------------------------------
void vtkTDxInteractorStyleCallback::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
}
