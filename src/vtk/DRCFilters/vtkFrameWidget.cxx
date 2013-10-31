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


  // Left mouse press/release for selecting the widget and starting
  // a rotation
  this->CallbackMapper->SetCallbackMethod(vtkCommand::LeftButtonPressEvent,
                                          vtkEvent::NoModifier,
                                          0, 0, NULL,
                                          vtkWidgetEvent::Select,
                                          this, vtkFrameWidget::SelectAction);

  this->CallbackMapper->SetCallbackMethod(vtkCommand::LeftButtonReleaseEvent,
                                          vtkEvent::NoModifier,
                                          0, 0, NULL,
                                          vtkWidgetEvent::EndSelect,
                                          this, vtkFrameWidget::EndSelectAction);

  // Start/end widget translation on middle mouse or left mouse + control or shift.
  this->CallbackMapper->SetCallbackMethod(vtkCommand::MiddleButtonPressEvent,
                                          vtkWidgetEvent::Select,
                                          this, vtkFrameWidget::SelectAction);

  this->CallbackMapper->SetCallbackMethod(vtkCommand::MiddleButtonReleaseEvent,
                                          vtkWidgetEvent::EndSelect,
                                          this, vtkFrameWidget::EndSelectAction);


  this->CallbackMapper->SetCallbackMethod(vtkCommand::LeftButtonPressEvent,
                                          vtkEvent::ShiftModifier,
                                          0, 0, NULL,
                                          vtkWidgetEvent::Select,
                                          this, vtkFrameWidget::SelectAction);

  this->CallbackMapper->SetCallbackMethod(vtkCommand::LeftButtonReleaseEvent,
                                            vtkEvent::ShiftModifier,
                                            0, 0, NULL,
                                          vtkWidgetEvent::EndSelect,
                                          this, vtkFrameWidget::EndSelectAction);

  // Catch mouse movement to send the movement deltas to the representation
  this->CallbackMapper->SetCallbackMethod(vtkCommand::MouseMoveEvent,
                                          vtkWidgetEvent::Move,
                                          this, vtkFrameWidget::MoveAction);
}

//----------------------------------------------------------------------------
vtkFrameWidget::~vtkFrameWidget()
{  
}

//----------------------------------------------------------------------
void vtkFrameWidget::SelectAction(vtkAbstractWidget *w)
{
  // We are in a static method, cast to ourself
  vtkFrameWidget *self = reinterpret_cast<vtkFrameWidget*>(w);

  // Get the event position
  int X = self->Interactor->GetEventPosition()[0];
  int Y = self->Interactor->GetEventPosition()[1];

  // Okay, make sure that the pick is in the current renderer
  if ( !self->CurrentRenderer ||
       !self->CurrentRenderer->IsInViewport(X,Y) )
    {
    self->WidgetState = vtkFrameWidget::Start;
    return;
    }

  // Begin the widget interaction which has the side effect of setting the
  // interaction state.
  double e[2];
  e[0] = static_cast<double>(X);
  e[1] = static_cast<double>(Y);
  self->WidgetRep->StartWidgetInteraction(e);
  int interactionState = self->WidgetRep->GetInteractionState();
  if ( interactionState == vtkFrameWidgetRepresentation::Outside )
    {
    return;
    }

  // We are definitely selected
  self->WidgetState = vtkFrameWidget::Active;
  self->GrabFocus(self->EventCallbackCommand);

  if (interactionState == vtkFrameWidgetRepresentation::Rotating && self->Interactor->GetShiftKey())
    {
    interactionState = vtkFrameWidgetRepresentation::TranslatingInPlane;
    }

  // The SetInteractionState has the side effect of highlighting the widget
  reinterpret_cast<vtkFrameWidgetRepresentation*>(self->WidgetRep)->
    SetInteractionState(interactionState);

  // start the interaction
  self->EventCallbackCommand->SetAbortFlag(1);
  self->StartInteraction();
  self->InvokeEvent(vtkCommand::StartInteractionEvent,NULL);
  self->Render();
}

//----------------------------------------------------------------------
void vtkFrameWidget::MoveAction(vtkAbstractWidget *w)
{
  vtkFrameWidget *self = reinterpret_cast<vtkFrameWidget*>(w);

  double e[2];
  e[0] = static_cast<double>(self->Interactor->GetEventPosition()[0]);
  e[1] = static_cast<double>(self->Interactor->GetEventPosition()[1]);

  // Return if we are not selected
  if (self->WidgetState == vtkFrameWidget::Start)
    {
    static_cast<vtkFrameWidgetRepresentation*>(self->WidgetRep)->OnMouseHover(e);
    self->Render();
    return;
    }


  // Do the widget interaction

  self->WidgetRep->WidgetInteraction(e);
  self->EventCallbackCommand->SetAbortFlag(1);
  self->InvokeEvent(vtkCommand::InteractionEvent,NULL);
  self->Render();
}

//----------------------------------------------------------------------
void vtkFrameWidget::EndSelectAction(vtkAbstractWidget *w)
{
  vtkFrameWidget *self = reinterpret_cast<vtkFrameWidget*>(w);
  if (self->WidgetState == vtkFrameWidget::Start)
    {
    return;
    }

  // Return state to not active
  self->WidgetState = vtkFrameWidget::Start;
  reinterpret_cast<vtkFrameWidgetRepresentation*>(self->WidgetRep)->
    SetInteractionState(vtkFrameWidgetRepresentation::Outside);

  reinterpret_cast<vtkFrameWidgetRepresentation*>(self->WidgetRep)->HighlightOff();

  self->ReleaseFocus();

  self->EventCallbackCommand->SetAbortFlag(1);
  self->EndInteraction();
  self->InvokeEvent(vtkCommand::EndInteractionEvent,NULL);
  self->Render();
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
