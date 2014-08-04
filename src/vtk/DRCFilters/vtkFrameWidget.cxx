/*=========================================================================

Program:   Visualization Toolkit

Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
All rights reserved.
See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

This software is distributed WITHOUT ANY WARRANTY; without even
the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkFrameWidget.h"
#include "vtkFrameWidgetRepresentation.h"

#include "vtkCommand.h"
#include "vtkCallbackCommand.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkObjectFactory.h"
#include "vtkWidgetEventTranslator.h"
#include "vtkWidgetCallbackMapper.h" 
#include "vtkEvent.h"
#include "vtkWidgetEvent.h"
#include "vtkRenderWindow.h"
#include "vtkRenderer.h"
#include "vtkAxesActor.h"


vtkStandardNewMacro(vtkFrameWidget);

//----------------------------------------------------------------------------
vtkFrameWidget::vtkFrameWidget()
{
  this->WidgetState = vtkFrameWidget::Start;
  this->ManagesCursor = 1;
  this->HandleRotationEnabled = true;

  this->CallbackMapper->SetCallbackMethod(vtkCommand::LeftButtonPressEvent,
                                          vtkEvent::NoModifier,
                                          0, 0, NULL,
                                          vtkWidgetEvent::Translate,
                                          this, vtkFrameWidget::TranslateAction);

  this->CallbackMapper->SetCallbackMethod(vtkCommand::LeftButtonReleaseEvent,
                                          vtkEvent::NoModifier,
                                          0, 0, NULL,
                                          vtkWidgetEvent::EndTranslate,
                                          this, vtkFrameWidget::EndTranslateAction);

  /*
  this->CallbackMapper->SetCallbackMethod(vtkCommand::LeftButtonPressEvent,
                                          vtkEvent::ShiftModifier,
                                          0, 0, NULL,
                                          vtkWidgetEvent::Rotate,
                                          this, vtkFrameWidget::RotateAction);

  this->CallbackMapper->SetCallbackMethod(vtkCommand::LeftButtonReleaseEvent,
                                            vtkEvent::ShiftModifier,
                                            0, 0, NULL,
                                          vtkWidgetEvent::EndRotate,
                                          this, vtkFrameWidget::EndRotateAction);
  */

  this->CallbackMapper->SetCallbackMethod(vtkCommand::RightButtonPressEvent,
                                          vtkWidgetEvent::Rotate,
                                          this, vtkFrameWidget::RotateAction);

  this->CallbackMapper->SetCallbackMethod(vtkCommand::RightButtonReleaseEvent,
                                          vtkWidgetEvent::EndRotate,
                                          this, vtkFrameWidget::EndRotateAction);

  this->CallbackMapper->SetCallbackMethod(vtkCommand::MouseMoveEvent,
                                          vtkWidgetEvent::Move,
                                          this, vtkFrameWidget::MoveAction);
}

//----------------------------------------------------------------------------
vtkFrameWidget::~vtkFrameWidget()
{
}

//----------------------------------------------------------------------
void vtkFrameWidget::OnEndInteraction()
{
  if (this->WidgetState == vtkFrameWidget::Start)
    {
    return;
    }

  this->WidgetState = vtkFrameWidget::Start;

  vtkFrameWidgetRepresentation* rep = vtkFrameWidgetRepresentation::SafeDownCast(this->WidgetRep);
  rep->SetInteractionState(vtkFrameWidgetRepresentation::Outside);

  this->UpdateMouseHover();

  this->ReleaseFocus();
  this->EventCallbackCommand->SetAbortFlag(1);
  this->EndInteraction();
  this->InvokeEvent(vtkCommand::EndInteractionEvent, NULL);
  this->Render();
}

//----------------------------------------------------------------------
void vtkFrameWidget::OnTranslate()
{
  int X = this->Interactor->GetEventPosition()[0];
  int Y = this->Interactor->GetEventPosition()[1];
  double e[2] = {static_cast<double>(X), static_cast<double>(Y)};

  if (!this->CurrentRenderer || !this->CurrentRenderer->IsInViewport(X,Y))
    {
    this->WidgetState = vtkFrameWidget::Start;
    return;
    }

  vtkFrameWidgetRepresentation* rep = vtkFrameWidgetRepresentation::SafeDownCast(this->WidgetRep);

  rep->StartWidgetInteraction(e);
  int interactionState = rep->GetInteractionState();
  if (interactionState == vtkFrameWidgetRepresentation::Outside)
    {
    return;
    }

  if (interactionState == vtkFrameWidgetRepresentation::Rotating)
    {
    interactionState = vtkFrameWidgetRepresentation::TranslatingInPlane;
    }

  rep->SetInteractionState(interactionState);
  this->WidgetState = vtkFrameWidget::Active;
  this->GrabFocus(this->EventCallbackCommand);
  this->EventCallbackCommand->SetAbortFlag(1);
  this->StartInteraction();
  this->InvokeEvent(vtkCommand::StartInteractionEvent, NULL);
  this->Render();
}

//----------------------------------------------------------------------
void vtkFrameWidget::OnRotate()
{
  int X = this->Interactor->GetEventPosition()[0];
  int Y = this->Interactor->GetEventPosition()[1];
  double e[2] = {static_cast<double>(X), static_cast<double>(Y)};

  if (!this->CurrentRenderer || !this->CurrentRenderer->IsInViewport(X,Y))
    {
    this->WidgetState = vtkFrameWidget::Start;
    return;
    }

  vtkFrameWidgetRepresentation* rep = vtkFrameWidgetRepresentation::SafeDownCast(this->WidgetRep);

  rep->StartWidgetInteraction(e);
  int interactionState = rep->GetInteractionState();
  if (interactionState == vtkFrameWidgetRepresentation::Outside)
    {
    return;
    }

  if (interactionState == vtkFrameWidgetRepresentation::Translating)
    {
    if (this->HandleRotationEnabled)
      {
      interactionState = vtkFrameWidgetRepresentation::Rotating;
      }
    else
      {
      rep->SetInteractionState(vtkFrameWidgetRepresentation::Outside);
      this->WidgetState = vtkFrameWidget::Start;
      return;
      }
    }

  rep->SetInteractionState(interactionState);
  this->WidgetState = vtkFrameWidget::Active;
  this->GrabFocus(this->EventCallbackCommand);
  this->EventCallbackCommand->SetAbortFlag(1);
  this->StartInteraction();
  this->InvokeEvent(vtkCommand::StartInteractionEvent, NULL);
  this->Render();
}

//----------------------------------------------------------------------
void vtkFrameWidget::UpdateMouseHover()
{
  double e[2] = {static_cast<double>(this->Interactor->GetEventPosition()[0]),
                 static_cast<double>(this->Interactor->GetEventPosition()[1])};

  vtkFrameWidgetRepresentation* rep = vtkFrameWidgetRepresentation::SafeDownCast(this->WidgetRep);
  rep->OnMouseHover(e);
  this->Render();
}

//----------------------------------------------------------------------
void vtkFrameWidget::OnMouseMove()
{
  if (this->WidgetState == vtkFrameWidget::Start)
    {
    this->UpdateMouseHover();
    return;
    }

  double e[2] = {static_cast<double>(this->Interactor->GetEventPosition()[0]),
                 static_cast<double>(this->Interactor->GetEventPosition()[1])};

  this->WidgetRep->WidgetInteraction(e);
  this->EventCallbackCommand->SetAbortFlag(1);
  this->InvokeEvent(vtkCommand::InteractionEvent, NULL);
  this->Render();
}

//----------------------------------------------------------------------
void vtkFrameWidget::MoveAction(vtkAbstractWidget *w)
{
  vtkFrameWidget::SafeDownCast(w)->OnMouseMove();
}

//----------------------------------------------------------------------
void vtkFrameWidget::TranslateAction(vtkAbstractWidget *w)
{
  vtkFrameWidget::SafeDownCast(w)->OnTranslate();
}

//----------------------------------------------------------------------
void vtkFrameWidget::RotateAction(vtkAbstractWidget *w)
{
  vtkFrameWidget::SafeDownCast(w)->OnRotate();
}

//----------------------------------------------------------------------
void vtkFrameWidget::EndRotateAction(vtkAbstractWidget *w)
{
  vtkFrameWidget::SafeDownCast(w)->OnEndInteraction();
}

//----------------------------------------------------------------------
void vtkFrameWidget::EndTranslateAction(vtkAbstractWidget *w)
{
  vtkFrameWidget::SafeDownCast(w)->OnEndInteraction();
}

//----------------------------------------------------------------------
void vtkFrameWidget::CreateDefaultRepresentation()
{
  if (!this->WidgetRep)
    {
    this->WidgetRep = vtkFrameWidgetRepresentation::New();
    }
}

//----------------------------------------------------------------------------
void vtkFrameWidget::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
}
