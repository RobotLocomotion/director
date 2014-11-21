/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkInteractorStyleTerrain2.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkInteractorStyleTerrain2.h"

#include "vtkActor.h"
#include "vtkCamera.h"
#include "vtkCallbackCommand.h"
#include "vtkExtractEdges.h"
#include "vtkMath.h"
#include "vtkObjectFactory.h"
#include "vtkPolyData.h"
#include "vtkPolyDataMapper.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkSphereSource.h"

vtkStandardNewMacro(vtkInteractorStyleTerrain2);

//----------------------------------------------------------------------------
vtkInteractorStyleTerrain2::vtkInteractorStyleTerrain2()
{
  this->LatLongLines = 0;

  this->LatLongSphere = NULL;
  this->LatLongExtractEdges = NULL;
  this->LatLongMapper = NULL;
  this->LatLongActor = NULL;

  this->MotionFactor   = 10.0;
}

//----------------------------------------------------------------------------
vtkInteractorStyleTerrain2::~vtkInteractorStyleTerrain2()
{
  if (this->LatLongSphere != NULL) 
    {
    this->LatLongSphere->Delete();
    }

  if (this->LatLongMapper != NULL) 
    {
    this->LatLongMapper->Delete();
    }

  if (this->LatLongActor != NULL) 
    {
    this->LatLongActor->Delete();
    }

  if (this->LatLongExtractEdges != NULL) 
    {
    this->LatLongExtractEdges->Delete();
    }
}

//----------------------------------------------------------------------------
void vtkInteractorStyleTerrain2::OnMouseMove() 
{ 
  int x = this->Interactor->GetEventPosition()[0];
  int y = this->Interactor->GetEventPosition()[1];

  switch (this->State) 
    {
    case VTKIS_ROTATE:
      this->FindPokedRenderer(x, y);
      this->Rotate();
      this->InvokeEvent(vtkCommand::InteractionEvent, NULL);
      break;

    case VTKIS_PAN:
      this->FindPokedRenderer(x, y);
      this->Pan();
      this->InvokeEvent(vtkCommand::InteractionEvent, NULL);
      break;

    case VTKIS_DOLLY:
      this->FindPokedRenderer(x, y);
      this->Dolly();
      this->InvokeEvent(vtkCommand::InteractionEvent, NULL);
      break;
    }
}

//----------------------------------------------------------------------------
void vtkInteractorStyleTerrain2::OnLeftButtonDown () 
{ 
  this->FindPokedRenderer(this->Interactor->GetEventPosition()[0], 
                          this->Interactor->GetEventPosition()[1]);
  if (this->CurrentRenderer == NULL)
    {
    return;
    }

  this->GrabFocus(this->EventCallbackCommand);

  vtkRenderWindowInteractor *rwi = this->Interactor;

  if (rwi->GetShiftKey())
  {
    this->StartPan();
  }
  else
  {
    this->StartRotate();
  }
}

//----------------------------------------------------------------------------
void vtkInteractorStyleTerrain2::OnLeftButtonUp ()
{
  switch (this->State) 
    {
    case VTKIS_ROTATE:
      this->EndRotate();
      if ( this->Interactor )
        {
        this->ReleaseFocus();
        }
      break;
    case VTKIS_PAN:
      this->EndPan();
      if ( this->Interactor )
        {
        this->ReleaseFocus();
        }
      break;
    }
}

//----------------------------------------------------------------------------
void vtkInteractorStyleTerrain2::OnMiddleButtonDown () 
{
  this->FindPokedRenderer(this->Interactor->GetEventPosition()[0], 
                          this->Interactor->GetEventPosition()[1]);
  if (this->CurrentRenderer == NULL)
    {
    return;
    }
  
  this->GrabFocus(this->EventCallbackCommand);
  this->StartPan();
}

//----------------------------------------------------------------------------
void vtkInteractorStyleTerrain2::OnMiddleButtonUp ()
{
  switch (this->State) 
    {
    case VTKIS_PAN:
      this->EndPan();
      if ( this->Interactor )
        {
        this->ReleaseFocus();
        }
      break;
    }
}

//----------------------------------------------------------------------------
void vtkInteractorStyleTerrain2::OnRightButtonDown () 
{
  this->FindPokedRenderer(this->Interactor->GetEventPosition()[0], 
                          this->Interactor->GetEventPosition()[1]);
  if (this->CurrentRenderer == NULL)
    {
    return;
    }
  
  this->GrabFocus(this->EventCallbackCommand);
  this->StartDolly();
}

//----------------------------------------------------------------------------
void vtkInteractorStyleTerrain2::OnRightButtonUp ()
{
  switch (this->State) 
    {
    case VTKIS_DOLLY:
      this->EndDolly();
      if ( this->Interactor )
        {
        this->ReleaseFocus();
        }
      break;
    }
}

//----------------------------------------------------------------------------
void vtkInteractorStyleTerrain2::Rotate()
{
  if (this->CurrentRenderer == NULL)
    {
    return;
    }

  vtkRenderWindowInteractor *rwi = this->Interactor;

  int dx = - ( rwi->GetEventPosition()[0] - rwi->GetLastEventPosition()[0] );
  int dy = - ( rwi->GetEventPosition()[1] - rwi->GetLastEventPosition()[1] );

  int *size = this->CurrentRenderer->GetRenderWindow()->GetSize();

  double a = dx / static_cast<double>( size[0]) * 180.0;
  double e = dy / static_cast<double>( size[1]) * 180.0;
  
  if (rwi->GetControlKey()) 
    {
    if(abs( dx ) >= abs( dy ))
      {
      e = 0.0;
      }
    else
      {
      a = 0.0;
      }
    }

  // Move the camera. 
  // Make sure that we don't hit the north pole singularity.

  vtkCamera *camera = this->CurrentRenderer->GetActiveCamera();
  camera->Azimuth( a );

  double dop[3], vup[3];

  camera->GetDirectionOfProjection( dop );
  vtkMath::Normalize( dop );
  camera->GetViewUp( vup );
  vtkMath::Normalize( vup );

  double angle = vtkMath::DegreesFromRadians( acos(vtkMath::Dot( dop, vup) ) );
  //printf("current angle: %.2f.  elvation delta: %.2f\n", angle, e);

  if ( ( angle + e ) > 177.0 ||
       ( angle + e ) < 3.0 )
    {
    //printf("  ...clamping elvation delta.\n");
    e = 0.0;
    }

  camera->Elevation( e );

  if ( this->AutoAdjustCameraClippingRange )
    {
    this->CurrentRenderer->ResetCameraClippingRange();
    }

  rwi->Render();
}

//----------------------------------------------------------------------------
void vtkInteractorStyleTerrain2::Pan()
{
  if (this->CurrentRenderer == NULL)
    {
    return;
    }

  vtkRenderWindowInteractor *rwi = this->Interactor;

  // Get the vector of motion

  double fp[3], focalPoint[3], pos[3], v[3], p1[4], p2[4];

  vtkCamera *camera = this->CurrentRenderer->GetActiveCamera();
  camera->GetPosition( pos );
  camera->GetFocalPoint( fp );

  this->ComputeWorldToDisplay(fp[0], fp[1], fp[2], 
                              focalPoint);


  int eventPos[2] = {rwi->GetEventPosition()[0], rwi->GetEventPosition()[1]};
  int lastEventPos[2] = {rwi->GetLastEventPosition()[0], rwi->GetLastEventPosition()[1]};


  if (rwi->GetControlKey())
    {
    int mouseDelta[2] = {eventPos[0] - lastEventPos[0], eventPos[1] - lastEventPos[1]};
    if(abs( mouseDelta[0] ) >= abs( mouseDelta[1] ))
      {
      eventPos[1] = lastEventPos[1];
      }
    else
      {
      eventPos[0] = lastEventPos[0];
      }
    }

  this->ComputeDisplayToWorld(eventPos[0], eventPos[1],
                              focalPoint[2],
                              p1);

  this->ComputeDisplayToWorld(lastEventPos[0], lastEventPos[1],
                              focalPoint[2],
                              p2);

  for (int i=0; i<3; i++)
    {
    v[i] = p2[i] - p1[i];
    pos[i] += v[i];
    fp[i] += v[i];
    }

  camera->SetPosition( pos );
  camera->SetFocalPoint( fp );

  if (rwi->GetLightFollowCamera()) 
    {
    this->CurrentRenderer->UpdateLightsGeometryToFollowCamera();
    }

  rwi->Render();
}

//----------------------------------------------------------------------------
void vtkInteractorStyleTerrain2::Dolly()
{
  if (this->CurrentRenderer == NULL)
    {
    return;
    }

  vtkRenderWindowInteractor *rwi = this->Interactor;
  vtkCamera *camera = this->CurrentRenderer->GetActiveCamera();
  double *center = this->CurrentRenderer->GetCenter();

  int dy = rwi->GetEventPosition()[1] - rwi->GetLastEventPosition()[1];
  double dyf = this->MotionFactor * dy / center[1];
  double zoomFactor = pow(1.1, dyf);
  
  if (camera->GetParallelProjection())
    {
    camera->SetParallelScale(camera->GetParallelScale() / zoomFactor);
    }
  else
    {
    camera->Dolly( zoomFactor );
    if (this->AutoAdjustCameraClippingRange)
      {
      this->CurrentRenderer->ResetCameraClippingRange();
      }
    }

  if (rwi->GetLightFollowCamera()) 
    {
    this->CurrentRenderer->UpdateLightsGeometryToFollowCamera();
    }
  
  rwi->Render();
}

//----------------------------------------------------------------------------
void vtkInteractorStyleTerrain2::OnChar()
{
  vtkRenderWindowInteractor *rwi = this->Interactor;

  switch (rwi->GetKeyCode())
    {
    case 'l':
      this->FindPokedRenderer(rwi->GetEventPosition()[0],
                              rwi->GetEventPosition()[1]);
      this->CreateLatLong();
      if (this->LatLongLines) 
        {
        this->LatLongLinesOff();
        }
      else 
        {
        double bounds[6];
        this->CurrentRenderer->ComputeVisiblePropBounds( bounds );
        double radius = sqrt((bounds[1]-bounds[0])*(bounds[1]-bounds[0]) +
                             (bounds[3]-bounds[2])*(bounds[3]-bounds[2]) +
                             (bounds[5]-bounds[4])*(bounds[5]-bounds[4])) /2.0;
        this->LatLongSphere->SetRadius( radius );
        this->LatLongSphere->SetCenter((bounds[0]+bounds[1])/2.0,
                                       (bounds[2]+bounds[3])/2.0,
                                       (bounds[4]+bounds[5])/2.0);        
        this->LatLongLinesOn();
        }
      this->SelectRepresentation();
      rwi->Render();
      break;

    default:
      this->Superclass::OnChar();
      break;
    }
}

//----------------------------------------------------------------------------
void vtkInteractorStyleTerrain2::CreateLatLong()
{
  if (this->LatLongSphere == NULL)
    {
    this->LatLongSphere = vtkSphereSource::New();
    this->LatLongSphere->SetPhiResolution( 13 );
    this->LatLongSphere->SetThetaResolution( 25 );
    this->LatLongSphere->LatLongTessellationOn();
    }

  if (this->LatLongExtractEdges == NULL)
    {
    this->LatLongExtractEdges = vtkExtractEdges::New();
    this->LatLongExtractEdges->SetInput(this->LatLongSphere->GetOutput());
    }

  if (this->LatLongMapper == NULL)
    {
    this->LatLongMapper = vtkPolyDataMapper::New();
    this->LatLongMapper->SetInput(this->LatLongExtractEdges->GetOutput());
    }

  if (this->LatLongActor == NULL)
    {
    this->LatLongActor = vtkActor::New();
    this->LatLongActor->SetMapper(this->LatLongMapper);
    this->LatLongActor->PickableOff();
    }
}

//----------------------------------------------------------------------------
void vtkInteractorStyleTerrain2::SelectRepresentation()
{
  if (this->CurrentRenderer == NULL)
    {
    return;
    }

  this->CurrentRenderer->RemoveActor(this->LatLongActor);
  
  if (this->LatLongLines)
    {
    this->CurrentRenderer->AddActor(this->LatLongActor);
    this->LatLongActor->VisibilityOn();
    }
  else
    {
    this->LatLongActor->VisibilityOff();
    }
}

//----------------------------------------------------------------------------
void vtkInteractorStyleTerrain2::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
  
  os << indent << "Latitude/Longitude Lines: " 
     << (this->LatLongLines ? "On\n" : "Off\n");
}


