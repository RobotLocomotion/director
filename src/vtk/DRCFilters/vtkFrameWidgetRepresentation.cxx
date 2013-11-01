/*=========================================================================

Program:   Visualization Toolkit

Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
All rights reserved.
See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

This software is distributed WITHOUT ANY WARRANTY; without even
the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkFrameWidgetRepresentation.h"

#include "vtkActor.h"
#include "vtkSphereSource.h"
#include "vtkPolyDataMapper.h"
#include "vtkPolyData.h"
#include "vtkCallbackCommand.h"
#include "vtkBox.h"
#include "vtkPolyData.h"
#include "vtkProperty.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkInteractorObserver.h"
#include "vtkMath.h"
#include "vtkCellArray.h"
#include "vtkCellPicker.h"
#include "vtkTransform.h"
#include "vtkDoubleArray.h"
#include "vtkBox.h"
#include "vtkPlanes.h"
#include "vtkCamera.h"
#include "vtkAssemblyPath.h"
#include "vtkWindow.h"
#include "vtkObjectFactory.h"
#include "vtkCaptionActor2D.h"
#include "vtkSphereSource.h"
#include "vtkSmartPointer.h"
#include "vtkAxesActor.h"
#include "vtkTransformPolyDataFilter.h"
#include "vtkCylinderSource.h"
#include "vtkDiskSource.h"
#include "vtkLineSource.h"
#include "vtkRegularPolygonSource.h"
#include "vtkExtractEdges.h"
#include "vtkTubeFilter.h"
#include "vtkCleanPolyData.h"
#include "vtkPointData.h"
#include "vtkPolyData.h"
#include "vtkCellLocator.h"
#include "vtkPlane.h"
#include "vtkArrowSource.h"

#include <cmath>

#if VTK_MAJOR_VERSION == 5
  #define SetInputData(x) SetInput(x)
#endif

vtkStandardNewMacro(vtkFrameWidgetRepresentation);

class DataRep
{
public:
  vtkSmartPointer<vtkActor>   Actor;
  vtkSmartPointer<vtkPolyDataMapper>   Mapper;
  vtkSmartPointer<vtkPolyData>   PolyData;
};

vtkSmartPointer<vtkPolyData> ShallowCopy(vtkPolyData* polyData)
{
  vtkSmartPointer<vtkPolyData> pd = vtkSmartPointer<vtkPolyData>::New();
  pd->ShallowCopy(polyData);
  return pd;
}

vtkSmartPointer<vtkPolyData> Transform(vtkPolyData* polyData, vtkTransform* t)
{
  vtkSmartPointer<vtkTransformPolyDataFilter> f = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
  f->SetTransform(t);
  f->SetInputData(polyData);
  f->Update();
  return ShallowCopy(f->GetOutput());
}

DataRep DataRepFromPolyData(vtkPolyData* polyData)
{
  DataRep rep;
  rep.PolyData = polyData;
  rep.Mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  rep.Actor = vtkSmartPointer<vtkActor>::New();
  rep.Mapper->SetInputData(rep.PolyData);
  rep.Actor->SetMapper(rep.Mapper);
  return rep;
}

DataRep MakeCircle(double radius, int axis)
{
  vtkSmartPointer<vtkRegularPolygonSource> c = vtkSmartPointer<vtkRegularPolygonSource>::New();
  c->GeneratePolygonOff();
  c->SetNumberOfSides(64);
  c->SetRadius(radius);
  c->SetCenter(0,0,0);
  c->Update();


  vtkSmartPointer<vtkTubeFilter> f = vtkSmartPointer<vtkTubeFilter>::New();
  f->SetInputConnection(c->GetOutputPort());
  f->SetRadius(0.01);
  f->SetNumberOfSides(24);
  f->Update();

  vtkSmartPointer<vtkTransform> t = vtkSmartPointer<vtkTransform>::New();
  if (axis == 0)
    {
    t->RotateY(90);
    }
  else if (axis == 1)
    {
    t->RotateX(90);
    }

  return DataRepFromPolyData(Transform(f->GetOutput(), t));
}


DataRep MakeAxis(double length, int axis)
{
  /*
  vtkSmartPointer<vtkArrowSource> c = vtkSmartPointer<vtkArrowSource>::New();
  c->SetTipResolution(32);
  c->SetShaftResolution(32);
  c->SetTipRadius(0.04);
  c->SetShaftRadius(0.01);
  c->SetTipLength(0.1);
  c->Update();
  */

  vtkSmartPointer<vtkLineSource> c = vtkSmartPointer<vtkLineSource>::New();
  c->SetPoint1(-length, 0, 0);
  c->SetPoint2(length, 0, 0);
  c->Update();

  vtkSmartPointer<vtkTubeFilter> f = vtkSmartPointer<vtkTubeFilter>::New();
  f->SetInputConnection(c->GetOutputPort());
  f->SetRadius(0.01);
  f->SetNumberOfSides(24);
  f->Update();


  vtkSmartPointer<vtkTransform> t = vtkSmartPointer<vtkTransform>::New();
  if (axis == 1)
    {
    t->RotateZ(90);
    }
  else if (axis == 2)
    {
    t->RotateY(-90);
    }

  return DataRepFromPolyData(Transform(f->GetOutput(), t));
}

DataRep MakeDisk(double radius, int axis)
{
  vtkSmartPointer<vtkDiskSource> d = vtkSmartPointer<vtkDiskSource>::New();
  d->SetCircumferentialResolution(32);
  d->SetOuterRadius(radius);
  d->SetInnerRadius(radius - 0.02);


  vtkSmartPointer<vtkTransform> t = vtkSmartPointer<vtkTransform>::New();
  if (axis == 0)
    {
    t->RotateY(90);
    }
  else if (axis == 2)
    {
    t->RotateX(90);
    }

  return DataRepFromPolyData(Transform(d->GetOutput(), t));
}

class vtkFrameWidgetRepresentation::vtkInternal {
public:
  vtkInternal()
    {
    this->RotateAxis = -1;
    this->TranslateAxis = -1;

    this->BoundingBox = vtkSmartPointer<vtkBox>::New();

    this->Transform = vtkSmartPointer<vtkTransform>::New();
    this->Transform->PostMultiply();

    this->SphereSource = vtkSmartPointer<vtkSphereSource>::New();
    this->SphereSource->SetRadius(0.05);
    this->SphereSource->Update();

    this->HoverMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    this->HoverMapper->SetInputConnection(this->SphereSource->GetOutputPort());

    this->HoverActor = vtkSmartPointer<vtkActor>::New();
    this->HoverActor->SetMapper(this->HoverMapper);
    this->HoverActor->VisibilityOff();
    }

  void InitPicker()
  {
    this->AxesPicker = vtkSmartPointer<vtkCellPicker>::New();
    this->AxesPicker->SetTolerance(0.01);
    this->AxesPicker->PickFromListOn();

    for (size_t i = 0; i < this->Reps.size(); ++i)
      {
      this->AxesPicker->AddPickList(Reps[i].Actor);
      }

    for (size_t i = 0; i < this->Axes.size(); ++i)
      {
      this->AxesPicker->AddPickList(Axes[i].Actor);
      }
  }

  void RebuildActors(double scale)
  {
    this->Reps.clear();
    this->Axes.clear();
    this->Actors.clear();

    Reps.push_back(MakeCircle(scale, 0));
    Reps.push_back(MakeCircle(scale, 1));
    Reps.push_back(MakeCircle(scale, 2));

    Axes.push_back(MakeAxis(scale, 0));
    Axes.push_back(MakeAxis(scale, 1));
    Axes.push_back(MakeAxis(scale, 2));

    Reps[0].Actor->GetProperty()->SetColor(1,0,0);
    Reps[1].Actor->GetProperty()->SetColor(0,1,0);
    Reps[2].Actor->GetProperty()->SetColor(0,0,1);
    Axes[0].Actor->GetProperty()->SetColor(1,0,0);
    Axes[1].Actor->GetProperty()->SetColor(0,1,0);
    Axes[2].Actor->GetProperty()->SetColor(0,0,1);

    for (size_t i = 0; i < this->Reps.size(); ++i)
      {
      Reps[i].Actor->SetUserTransform(this->Transform);
      Reps[i].Actor->GetProperty()->SetOpacity(1.0);
      this->Actors.push_back(Reps[i].Actor);
      }

    for (size_t i = 0; i < this->Axes.size(); ++i)
      {
      Axes[i].Actor->SetUserTransform(this->Transform);
      Axes[i].Actor->GetProperty()->SetOpacity(1.0);
      this->Actors.push_back(Axes[i].Actor);
      }

    //this->Actors.push_back(this->HoverActor);
    this->InitPicker();
  }

  vtkSmartPointer<vtkActor>   HoverActor;
  vtkSmartPointer<vtkPolyDataMapper>   HoverMapper;
  vtkSmartPointer<vtkSphereSource>   SphereSource;

  vtkSmartPointer<vtkCellLocator> CellLocator;

  vtkSmartPointer<vtkTransform>   Transform;
  vtkSmartPointer<vtkCellPicker>  AxesPicker;
  vtkSmartPointer<vtkBox>         BoundingBox;

  std::vector<DataRep> Reps;
  std::vector<DataRep> Axes;

  std::vector<vtkActor*> Actors;

  int TranslateAxis;
  int RotateAxis;
  double CirclePickPoint[3];
  double LastOrientation[4];
};

//----------------------------------------------------------------------------
vtkFrameWidgetRepresentation::vtkFrameWidgetRepresentation()
{
  this->Internal = new vtkInternal;
  this->InteractionState = vtkFrameWidgetRepresentation::Outside;
  this->WorldSize = 1.0;
  this->Internal->RebuildActors(this->WorldSize);
}

//----------------------------------------------------------------------------
vtkFrameWidgetRepresentation::~vtkFrameWidgetRepresentation()
{
  delete this->Internal;
}

//----------------------------------------------------------------------
void vtkFrameWidgetRepresentation::StartWidgetInteraction(double e[2])
{
  // Store the start position
  this->StartEventPosition[0] = e[0];
  this->StartEventPosition[1] = e[1];
  this->StartEventPosition[2] = 0.0;

  // Store the start position
  this->LastEventPosition[0] = e[0];
  this->LastEventPosition[1] = e[1];
  this->LastEventPosition[2] = 0.0;

  this->ComputeInteractionState(static_cast<int>(e[0]),static_cast<int>(e[1]),0);
}

//----------------------------------------------------------------------
void vtkFrameWidgetRepresentation::WidgetInteraction(double e[2])
{
  // Convert events to appropriate coordinate systems
  vtkCamera *camera = this->Renderer->GetActiveCamera();
  if ( !camera )
    {
    return;
    }

  double focalPoint[4], pickPoint[4], prevPickPoint[4];
  double z, vpn[3];
  camera->GetViewPlaneNormal(vpn);

  // Compute the two points defining the motion vector
  double pos[3];
  this->Internal->AxesPicker->GetPickPosition(pos);

  vtkInteractorObserver::ComputeWorldToDisplay(this->Renderer,
                                               pos[0], pos[1], pos[2],
                                               focalPoint);
  z = focalPoint[2];
  vtkInteractorObserver::ComputeDisplayToWorld(
                          this->Renderer,this->LastEventPosition[0],
                          this->LastEventPosition[1], z, prevPickPoint);

  vtkInteractorObserver::ComputeDisplayToWorld(this->Renderer, e[0], e[1],
                                                            z, pickPoint);


  if ( this->InteractionState == vtkFrameWidgetRepresentation::Translating )
    {
    this->Translate(prevPickPoint, pickPoint);
    }
  else if ( this->InteractionState == vtkFrameWidgetRepresentation::TranslatingInPlane )
    {
    this->TranslateInPlane(prevPickPoint, pickPoint);
    }
  else if ( this->InteractionState == vtkFrameWidgetRepresentation::Rotating )
    {
    this->Rotate(static_cast<int>(e[0]), static_cast<int>(e[1]),
                                  prevPickPoint, pickPoint, vpn);
    }

  // Store the start position
  this->LastEventPosition[0] = e[0];
  this->LastEventPosition[1] = e[1];
  this->LastEventPosition[2] = 0.0;
}

//----------------------------------------------------------------------------
void vtkFrameWidgetRepresentation::Translate(double *p1, double *p2)
{
  double axis[3] = {0,0,0};
  axis[this->Internal->TranslateAxis] = 1;
  this->Internal->Transform->TransformVector(axis, axis);

  double v[3] = {p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2]};

  vtkMath::ProjectVector(v, axis, v);
  this->Internal->Transform->Translate(v[0], v[1], v[2]);
}

//----------------------------------------------------------------------------
void vtkFrameWidgetRepresentation::TranslateInPlane(double *p1, double *p2)
{
  double planeOrigin[3] = {0,0,0};
  double planeNormal[3] = {0,0,0};
  planeNormal[this->Internal->RotateAxis] = 1;

  this->Internal->Transform->TransformPoint(planeOrigin, planeOrigin);
  this->Internal->Transform->TransformVector(planeNormal, planeNormal);

  vtkPlane::ProjectPoint(p1, planeOrigin, planeNormal, p1);
  vtkPlane::ProjectPoint(p2, planeOrigin, planeNormal, p2);
  double v[3] = {p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2]};

  this->Internal->Transform->Translate(v[0], v[1], v[2]);
}

//----------------------------------------------------------------------------
double ComputeSignedAngle(double v1[3], double v2[3], double normalVector[3])
{
  vtkMath::Normalize(v1);
  vtkMath::Normalize(v2);
  vtkMath::Normalize(normalVector);
  double v1_cross_v2[3];
  vtkMath::Cross(v1, v2, v1_cross_v2);
  return vtkMath::DegreesFromRadians(std::atan2(vtkMath::Dot(normalVector, v1_cross_v2),
                    vtkMath::Dot(v1, v2)));
}


//----------------------------------------------------------------------------
void vtkFrameWidgetRepresentation::Rotate(int X,
                                  int Y,
                                  double *p1,
                                  double *p2,
                                  double *vpn)
{
  double v[3]; //vector of motion
  double axis[3]; //axis of rotation
  double theta; //rotation angle

  v[0] = p2[0] - p1[0];
  v[1] = p2[1] - p1[1];
  v[2] = p2[2] - p1[2];

  // Create axis of rotation and angle of rotation
  vtkMath::Cross(vpn,v,axis);
  if ( vtkMath::Normalize(axis) == 0.0 )
    {
    return;
    }
  int *size = this->Renderer->GetSize();

  double l2 = (X-this->LastEventPosition[0])*(X-this->LastEventPosition[0])
             + (Y-this->LastEventPosition[1])*(Y-this->LastEventPosition[1]);
  theta = 360.0 * sqrt(l2/(size[0]*size[0]+size[1]*size[1]));


  // Translate our transform back to the origin, do the rotation, then move
  // it back to the current position
  double pos[3];
  this->Internal->Transform->GetPosition(pos);



  if (this->Internal->RotateAxis >= 0)
    {


    double rayPoint0[4];
    double rayPoint1[4];
    double t;
    double planePoint[3];
    double closestPoint[3];
    vtkIdType cellId;
    int subId;
    double dist;

    vtkInteractorObserver::ComputeDisplayToWorld(this->Renderer, X, Y, 0.0, rayPoint0);
    vtkInteractorObserver::ComputeDisplayToWorld(this->Renderer, X, Y, 1.0, rayPoint1);

    this->Internal->Transform->GetLinearInverse()->TransformPoint(rayPoint0, rayPoint0);
    this->Internal->Transform->GetLinearInverse()->TransformPoint(rayPoint1, rayPoint1);

    double circleOrigin[3] = {0,0,0};
    double circleNormal[3] = {0,0,0};
    circleNormal[this->Internal->RotateAxis] = 1;

    vtkPlane::IntersectWithLine(rayPoint0, rayPoint1, circleNormal, circleOrigin, t, planePoint);
    this->Internal->CellLocator->FindClosestPoint(planePoint, closestPoint, cellId, subId, dist);
    this->Internal->Transform->TransformPoint(closestPoint, closestPoint);

    this->Internal->SphereSource->SetCenter(closestPoint);

    //printf("pick point: %f %f %f\n", this->Internal->CirclePickPoint[0], this->Internal->CirclePickPoint[1], this->Internal->CirclePickPoint[2]);
    //printf("closest point: %f %f %f\n", closestPoint[0], closestPoint[1], closestPoint[2]);

    this->Internal->Transform->TransformVector(circleNormal, circleNormal);
    this->Internal->Transform->TransformPoint(circleOrigin, circleOrigin);

    double edge1[3] = {this->Internal->CirclePickPoint[0] - circleOrigin[0],
                       this->Internal->CirclePickPoint[1] - circleOrigin[1],
                       this->Internal->CirclePickPoint[2] - circleOrigin[2] };

    double edge2[3] = {closestPoint[0] - circleOrigin[0],
                       closestPoint[1] - circleOrigin[1],
                       closestPoint[2] - circleOrigin[2] };

    theta = ComputeSignedAngle(edge1, edge2, circleNormal);
    //printf("theta: %f\n", theta);

    this->Internal->Transform->Identity();
    this->Internal->Transform->RotateWXYZ(this->Internal->LastOrientation[0], this->Internal->LastOrientation+1);
    this->Internal->Transform->RotateWXYZ(theta, circleNormal);

    }
  else
    {

    this->Internal->Transform->Translate(-pos[0], -pos[1], -pos[2]);
    this->Internal->Transform->RotateWXYZ(theta,axis);

    }

  this->Internal->Transform->Translate(pos[0], pos[1], pos[2]);
}

//----------------------------------------------------------------------------
vtkTransform* vtkFrameWidgetRepresentation::GetTransform()
{
  return this->Internal->Transform;
}

//----------------------------------------------------------------------------
void vtkFrameWidgetRepresentation::SetTransform(vtkTransform* t)
{
  if (!t)
    {
    vtkErrorMacro(<<"vtkTransform must be non-NULL");
    return;
    }

  this->Internal->Transform = t;
  this->Internal->Transform->PostMultiply();
  for (size_t i = 0; i < this->Internal->Actors.size(); ++i)
    {
    this->Internal->Actors[i]->SetUserTransform(t);
    }
}

//----------------------------------------------------------------------------
void vtkFrameWidgetRepresentation::HighlightOff()
{
  this->HighlightActor(0);
}

//----------------------------------------------------------------------------
void vtkFrameWidgetRepresentation::HighlightActor(vtkDataSet* dataset)
{
  for (size_t i = 0; i < this->Internal->Reps.size(); ++i)
    {
    if (this->Internal->Reps[i].PolyData.GetPointer() == dataset)
      {
      this->Internal->Reps[i].Actor->GetProperty()->SetOpacity(1.0);
      this->Internal->Reps[i].Actor->GetProperty()->SetLineWidth(4.0);
      }
    else
      {
      this->Internal->Reps[i].Actor->GetProperty()->SetOpacity(1.0);
      this->Internal->Reps[i].Actor->GetProperty()->SetLineWidth(1.0);
      }
    }

  for (size_t i = 0; i < this->Internal->Axes.size(); ++i)
    {
    if (this->Internal->Axes[i].PolyData.GetPointer() == dataset)
      {
      this->Internal->Axes[i].Actor->GetProperty()->SetOpacity(1.0);
      this->Internal->Axes[i].Actor->GetProperty()->SetLineWidth(4.0);
      }
    else
      {
      this->Internal->Axes[i].Actor->GetProperty()->SetOpacity(1.0);
      this->Internal->Axes[i].Actor->GetProperty()->SetLineWidth(1.0);
      }
    }
}

//----------------------------------------------------------------------------
void vtkFrameWidgetRepresentation::OnMouseHover(double e[2])
{
  this->Internal->AxesPicker->Pick(e[0], e[1], 0.0, this->Renderer);
  vtkDataSet* dataset = this->Internal->AxesPicker->GetDataSet();
  this->HighlightActor(dataset);

}

//----------------------------------------------------------------------------
int vtkFrameWidgetRepresentation::ComputeInteractionState(int X, int Y, int vtkNotUsed(modify))
{
  this->InteractionState = vtkFrameWidgetRepresentation::Outside;
  if (!this->Renderer || !this->Renderer->IsInViewport(X, Y))
    {
    return this->InteractionState;
    }

  this->Internal->RotateAxis = -1;
  this->Internal->TranslateAxis = -1;

  // Check if the axes actor was picked
  this->Internal->AxesPicker->Pick(X,Y,0.0,this->Renderer);
  vtkDataSet* dataset = this->Internal->AxesPicker->GetDataSet();
  if (dataset)
    {


    for (size_t i = 0; i < this->Internal->Reps.size(); ++i)
      {
      if (this->Internal->Reps[i].PolyData.GetPointer() == dataset)
        {

        this->InteractionState = vtkFrameWidgetRepresentation::Rotating;

        printf("picked circle %d\n", i);
        this->Internal->RotateAxis = i;
        this->Internal->AxesPicker->GetPickPosition(this->Internal->CirclePickPoint);
        this->Internal->Transform->GetOrientationWXYZ(this->Internal->LastOrientation);

        this->Internal->CellLocator = vtkSmartPointer<vtkCellLocator>::New();
        this->Internal->CellLocator->SetDataSet(this->Internal->Reps[i].PolyData);
        this->Internal->CellLocator->BuildLocator();
        }
      }

    for (size_t i = 0; i < this->Internal->Axes.size(); ++i)
      {
      if (this->Internal->Axes[i].PolyData.GetPointer() == dataset)
        {
        printf("picked axis %d\n", i);
        this->Internal->TranslateAxis = i;

        this->InteractionState = vtkFrameWidgetRepresentation::Translating;
        }
      }


    }

  return this->InteractionState;
}

//----------------------------------------------------------------------
void vtkFrameWidgetRepresentation::SetInteractionState(int state)
{
  // Clamp to allowable values
  if (state < vtkFrameWidgetRepresentation::Outside)
    {
    state = vtkFrameWidgetRepresentation::Outside;
    }
  else if (state > vtkFrameWidgetRepresentation::Rotating)
    {
    state = vtkFrameWidgetRepresentation::Rotating;
    }

  this->InteractionState = state;
}

//----------------------------------------------------------------------
double *vtkFrameWidgetRepresentation::GetBounds()
{
  this->BuildRepresentation();


  for (size_t i = 0; i < this->Internal->Actors.size(); ++i)
    {
    double* b = this->Internal->Actors[i]->GetBounds();
    if (i == 0)
      {
      this->Internal->BoundingBox->SetBounds(b);
      }
    else
      {
      this->Internal->BoundingBox->AddBounds(b);
      }
    }

  return this->Internal->BoundingBox->GetBounds();
}

//----------------------------------------------------------------------------
void vtkFrameWidgetRepresentation::BuildRepresentation()
{
  // Rebuild only if necessary
  if ( this->GetMTime() > this->BuildTime ||
       (this->Renderer && this->Renderer->GetVTKWindow() &&
        (this->Renderer->GetVTKWindow()->GetMTime() > this->BuildTime ||
        this->Renderer->GetActiveCamera()->GetMTime() > this->BuildTime)) )
    {
    this->BuildTime.Modified();
    this->Internal->RebuildActors(this->WorldSize);
    }
}


//----------------------------------------------------------------------------
void vtkFrameWidgetRepresentation::ReleaseGraphicsResources(vtkWindow *w)
{
  for (size_t i = 0; i < this->Internal->Actors.size(); ++i)
    {
    this->Internal->Actors[i]->ReleaseGraphicsResources(w);
    }
}

//----------------------------------------------------------------------------
int vtkFrameWidgetRepresentation::RenderOpaqueGeometry(vtkViewport *v)
{
  int count=0;
  this->BuildRepresentation();

  for (size_t i = 0; i < this->Internal->Actors.size(); ++i)
    {
    count += this->Internal->Actors[i]->RenderOpaqueGeometry(v);
    }

  return count;
}

//----------------------------------------------------------------------------
int vtkFrameWidgetRepresentation::RenderOverlay(vtkViewport *v)
{
  int count=0;
  this->BuildRepresentation();

  for (size_t i = 0; i < this->Internal->Actors.size(); ++i)
    {
    count += this->Internal->Actors[i]->RenderOverlay(v);
    }

  return count;
}

//----------------------------------------------------------------------------
int vtkFrameWidgetRepresentation::RenderTranslucentPolygonalGeometry(vtkViewport *v)
{
  int count=0;
  this->BuildRepresentation();

  for (size_t i = 0; i < this->Internal->Actors.size(); ++i)
    {
    count += this->Internal->Actors[i]->RenderTranslucentPolygonalGeometry(v);
    }

  return count;
}

//----------------------------------------------------------------------------
int vtkFrameWidgetRepresentation::HasTranslucentPolygonalGeometry()
{
  int result = 0;
  this->BuildRepresentation();

  for (size_t i = 0; i < this->Internal->Actors.size(); ++i)
    {
    result |= this->Internal->Actors[i]->HasTranslucentPolygonalGeometry();
    }

  return result;
}

//----------------------------------------------------------------------------
void vtkFrameWidgetRepresentation::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);

  os << indent << "Transform: " << this->Internal->Transform << "\n";
}
