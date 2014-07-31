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
#include "vtkLine.h"
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
#include "vtkArcSource.h"
#include "vtkDiskSource.h"
#include "vtkLineSource.h"
#include "vtkRegularPolygonSource.h"
#include "vtkSphereSource.h"
#include "vtkAppendPolyData.h"
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

DataRep MakeCircle(double radius, int axis, bool useTubeFilter)
{

  vtkSmartPointer<vtkRegularPolygonSource> c = vtkSmartPointer<vtkRegularPolygonSource>::New();
  c->GeneratePolygonOff();
  c->SetNumberOfSides(64);
  c->SetRadius(radius);
  c->SetCenter(0,0,0);
  c->Update();


/*
  vtkSmartPointer<vtkArcSource> c = vtkSmartPointer<vtkArcSource>::New();
  c->SetResolution(32);
  c->SetCenter(0,0,0);
  c->SetPoint1(radius,0,0);
  c->SetPoint2(0,radius,0);
  c->Update();
*/

  vtkSmartPointer<vtkTubeFilter> f = vtkSmartPointer<vtkTubeFilter>::New();
  f->SetInputConnection(c->GetOutputPort());
  f->SetRadius(0.0025);
  f->SetNumberOfSides(24);
  f->Update();

  vtkPolyData* polyData = useTubeFilter ? f->GetOutput() : c->GetOutput();

  vtkSmartPointer<vtkTransform> t = vtkSmartPointer<vtkTransform>::New();
  if (axis == 0)
    {
    t->RotateY(-90);
    }
  else if (axis == 1)
    {
    t->RotateX(90);
    }

  return DataRepFromPolyData(Transform(polyData, t));
}


DataRep MakeAxis(double length, int axis, bool useTubeFilter)
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
  //c->SetPoint1(-length, 0, 0);
  c->SetPoint1(0, 0, 0);
  c->SetPoint2(length + length*0.5, 0, 0);
  c->Update();

  vtkSmartPointer<vtkTubeFilter> f = vtkSmartPointer<vtkTubeFilter>::New();
  f->SetInputConnection(c->GetOutputPort());
  f->SetRadius(0.0025);
  f->SetNumberOfSides(24);
  f->Update();

  vtkPolyData* polyData = useTubeFilter ? f->GetOutput() : c->GetOutput();

  vtkSmartPointer<vtkSphereSource> sphere = vtkSmartPointer<vtkSphereSource>::New();
  sphere->SetCenter(length + length*0.5, 0, 0);
  sphere->SetRadius(0.02);
  sphere->SetThetaResolution(32);
  sphere->SetPhiResolution(32);
  sphere->Update();

  vtkSmartPointer<vtkAppendPolyData> append = vtkSmartPointer<vtkAppendPolyData>::New();
  append->AddInputConnection(f->GetOutputPort());
  append->AddInputConnection(sphere->GetOutputPort());
  append->Update();

  polyData = useTubeFilter ? append->GetOutput() : c->GetOutput();

  vtkSmartPointer<vtkTransform> t = vtkSmartPointer<vtkTransform>::New();
  if (axis == 1)
    {
    t->RotateZ(90);
    }
  else if (axis == 2)
    {
    t->RotateY(-90);
    }



  return DataRepFromPolyData(Transform(polyData, t));
}

DataRep MakeDisk(double radius, int axis)
{
  vtkSmartPointer<vtkDiskSource> d = vtkSmartPointer<vtkDiskSource>::New();
  d->SetCircumferentialResolution(64);
  d->SetOuterRadius(radius);
  d->SetInnerRadius(radius - 0.01);


  vtkSmartPointer<vtkTransform> t = vtkSmartPointer<vtkTransform>::New();
  if (axis == 0)
    {
    t->RotateY(90);
    }
  else if (axis == 1)
    {
    t->RotateX(90);
    }

  return DataRepFromPolyData(Transform(d->GetOutput(), t));
}

class vtkFrameWidgetRepresentation::vtkInternal {
public:
  vtkInternal()
    {
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

  void RebuildActors(double scale, bool useTubeFilter)
  {
    this->Reps.clear();
    this->Axes.clear();
    this->Actors.clear();

    useTubeFilter = true;
    Reps.push_back(MakeCircle(scale, 0, useTubeFilter));
    Reps.push_back(MakeCircle(scale, 1, useTubeFilter));
    Reps.push_back(MakeCircle(scale, 2, useTubeFilter));


    /*
    Reps.push_back(MakeDisk(scale, 0));
    Reps.push_back(MakeDisk(scale, 1));
    Reps.push_back(MakeDisk(scale, 2));
    */

    Axes.push_back(MakeAxis(scale, 0, useTubeFilter));
    Axes.push_back(MakeAxis(scale, 1, useTubeFilter));
    Axes.push_back(MakeAxis(scale, 2, useTubeFilter));

    /*
    Reps[0].Actor->GetProperty()->SetColor(1,1,1);
    Reps[1].Actor->GetProperty()->SetColor(1,1,1);
    Reps[2].Actor->GetProperty()->SetColor(1,1,1);
    */


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

  vtkSmartPointer<vtkTransform>   Transform;
  vtkSmartPointer<vtkCellPicker>  AxesPicker;
  vtkSmartPointer<vtkBox>         BoundingBox;

  std::vector<DataRep> Reps;
  std::vector<DataRep> Axes;

  std::vector<vtkActor*> Actors;
};

//----------------------------------------------------------------------------
vtkFrameWidgetRepresentation::vtkFrameWidgetRepresentation()
{
  this->Internal = new vtkInternal;
  this->InteractionState = vtkFrameWidgetRepresentation::Outside;
  this->WorldSize = 0.5;
  this->UseTubeFilter = false;
  this->RotateAxis = -1;
  this->TranslateAxis = -1;
  this->InteractionStartWorldPoint[0] = 0.0;
  this->InteractionStartWorldPoint[1] = 0.0;
  this->InteractionStartWorldPoint[2] = 0.0;
  this->LastEventPosition[0] = 0.0;
  this->LastEventPosition[1] = 0.0;
  this->Internal->RebuildActors(this->WorldSize, this->UseTubeFilter);
}

//----------------------------------------------------------------------------
vtkFrameWidgetRepresentation::~vtkFrameWidgetRepresentation()
{
  delete this->Internal;
}

//----------------------------------------------------------------------
void vtkFrameWidgetRepresentation::SetTranslateAxisEnabled(int axisId, bool enabled)
{
  if (axisId < 0 || axisId > 2)
    {
    return;
    }

  this->Internal->Axes[axisId].Actor->SetVisibility(enabled);
}

//----------------------------------------------------------------------
void vtkFrameWidgetRepresentation::SetRotateAxisEnabled(int axisId, bool enabled)
{
  if (axisId < 0 || axisId > 2)
    {
    return;
    }

  this->Internal->Reps[axisId].Actor->SetVisibility(enabled);
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

  this->ComputeInteractionState(static_cast<int>(e[0]),static_cast<int>(e[1]),0);
}

//----------------------------------------------------------------------
void vtkFrameWidgetRepresentation::WidgetInteraction(double e[2])
{
  if ( this->InteractionState == vtkFrameWidgetRepresentation::Translating )
    {
    this->Translate(e);
    }
  else if ( this->InteractionState == vtkFrameWidgetRepresentation::TranslatingInPlane )
    {
    this->TranslateInPlane(e);
    }
  else if ( this->InteractionState == vtkFrameWidgetRepresentation::Rotating )
    {
    this->Rotate(e);
    }

  this->LastEventPosition[0] = e[0];
  this->LastEventPosition[1] = e[1];
}

//----------------------------------------------------------------------------
void vtkFrameWidgetRepresentation::Translate(double e[2])
{
  double uu, vv;

  double origin[3] = {0.0, 0.0, 0.0};
  double translateAxis[3] = {0.0, 0.0, 0.0};
  translateAxis[this->TranslateAxis] = 1.0;
  this->Internal->Transform->TransformVector(translateAxis, translateAxis);

  double linePoint[3];
  double prevLinePoint[3];

  double rayP0[3];
  double rayP1[3];
  double displayPoint[2];

  displayPoint[0] = e[0];
  displayPoint[1] = e[1];

  double lineEnd[3];
  lineEnd[0] = this->InteractionStartWorldPoint[0] + translateAxis[0];
  lineEnd[1] = this->InteractionStartWorldPoint[1] + translateAxis[1];
  lineEnd[2] = this->InteractionStartWorldPoint[2] + translateAxis[2];

  vtkInteractorObserver::ComputeDisplayToWorld(
                          this->Renderer,
                          displayPoint[0],
                          displayPoint[1], 0, rayP0);

  vtkInteractorObserver::ComputeDisplayToWorld(
                          this->Renderer,
                          displayPoint[0],
                          displayPoint[1], 1, rayP1);


  vtkLine::Intersection(rayP0, rayP1,
		this->InteractionStartWorldPoint, lineEnd,
		uu, vv);

  linePoint[0] = this->InteractionStartWorldPoint[0] + translateAxis[0] * vv;
  linePoint[1] = this->InteractionStartWorldPoint[1] + translateAxis[1] * vv;
  linePoint[2] = this->InteractionStartWorldPoint[2] + translateAxis[2] * vv;

  //

  displayPoint[0] = this->LastEventPosition[0];
  displayPoint[1] = this->LastEventPosition[1];

  vtkInteractorObserver::ComputeDisplayToWorld(
                          this->Renderer,
                          displayPoint[0],
                          displayPoint[1], 0, rayP0);

  vtkInteractorObserver::ComputeDisplayToWorld(
                          this->Renderer,
                          displayPoint[0],
                          displayPoint[1], 1, rayP1);


  vtkLine::Intersection(rayP0, rayP1,
		this->InteractionStartWorldPoint, lineEnd,
		uu, vv);


  prevLinePoint[0] = this->InteractionStartWorldPoint[0] + translateAxis[0] * vv;
  prevLinePoint[1] = this->InteractionStartWorldPoint[1] + translateAxis[1] * vv;
  prevLinePoint[2] = this->InteractionStartWorldPoint[2] + translateAxis[2] * vv;


  double worldDelta[3];
  worldDelta[0] = linePoint[0] - prevLinePoint[0];
  worldDelta[1] = linePoint[1] - prevLinePoint[1];
  worldDelta[2] = linePoint[2] - prevLinePoint[2];

  this->Internal->Transform->Translate(worldDelta[0], worldDelta[1], worldDelta[2]);
  this->Internal->Transform->Modified();
}


//----------------------------------------------------------------------------
void vtkFrameWidgetRepresentation::TranslateInPlane(double e[2])
{
  double t;
  double planePoint[3];
  double prevPlanePoint[3];

  double planeNormal[3] = {0,0,0};
  planeNormal[this->RotateAxis] = 1.0;
  this->Internal->Transform->TransformVector(planeNormal, planeNormal);

  double rayP0[3];
  double rayP1[3];
  double displayPoint[2];


  displayPoint[0] = e[0];
  displayPoint[1] = e[1];

  vtkInteractorObserver::ComputeDisplayToWorld(
                          this->Renderer,
                          displayPoint[0],
                          displayPoint[1], 0, rayP0);

  vtkInteractorObserver::ComputeDisplayToWorld(
                          this->Renderer,
                          displayPoint[0],
                          displayPoint[1], 1, rayP1);

  vtkPlane::IntersectWithLine(rayP0, rayP1, planeNormal, this->InteractionStartWorldPoint, t, planePoint);



  displayPoint[0] = this->LastEventPosition[0];
  displayPoint[1] = this->LastEventPosition[1];

  vtkInteractorObserver::ComputeDisplayToWorld(
                          this->Renderer,
                          displayPoint[0],
                          displayPoint[1], 0, rayP0);

  vtkInteractorObserver::ComputeDisplayToWorld(
                          this->Renderer,
                          displayPoint[0],
                          displayPoint[1], 1, rayP1);

  vtkPlane::IntersectWithLine(rayP0, rayP1, planeNormal, this->InteractionStartWorldPoint, t, prevPlanePoint);


  double worldDelta[3];
  worldDelta[0] = planePoint[0] - prevPlanePoint[0];
  worldDelta[1] = planePoint[1] - prevPlanePoint[1];
  worldDelta[2] = planePoint[2] - prevPlanePoint[2];

  this->Internal->Transform->Translate(worldDelta[0], worldDelta[1], worldDelta[2]);
  this->Internal->Transform->Modified();
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
void vtkFrameWidgetRepresentation::Rotate(double e[2])
{
  double centerOfRotation[3];
  this->Internal->Transform->GetPosition(centerOfRotation);

  double vpn[3];
  vtkCamera *camera = this->Renderer->GetActiveCamera();
  camera->GetViewPlaneNormal(vpn);

  if (this->RotateAxis >= 0)
    {

    // Compute the center of rotation in display coordinates
    double displayCenterOfRotation[3];
    vtkInteractorObserver::ComputeWorldToDisplay(this->Renderer,
                                                  centerOfRotation[0],
                                                  centerOfRotation[1],
                                                  centerOfRotation[2],
                                                  displayCenterOfRotation);

    // Compute rotation angle
    double vec1[3];
    vec1[0] = e[0] - displayCenterOfRotation[0];
    vec1[1] = e[1] - displayCenterOfRotation[1];
    vec1[2] = 0.0;

    double vec2[3];
    vec2[0] = this->LastEventPosition[0] - displayCenterOfRotation[0];
    vec2[1] = this->LastEventPosition[1] - displayCenterOfRotation[1];
    vec2[2] = 0.0;

    if (vtkMath::Normalize(vec1) == 0.0 || vtkMath::Normalize(vec2) == 0.0)
      {
      return;
      }

    double vectorDot = vtkMath::Dot(vec1, vec2);
    vectorDot = vectorDot > 1.0 ? 1.0 : vectorDot < -1.0 ? -1.0 : vectorDot;
    double theta = vtkMath::DegreesFromRadians(std::acos(vectorDot));

    double direction[3];
    vtkMath::Cross(vec1, vec2, direction);
    if (direction[2] < 0.0)
      {
      theta = -theta;
      }

    double rotateAxis[3] = {0,0,0};
    rotateAxis[this->RotateAxis] = 1;
    this->Internal->Transform->TransformVector(rotateAxis, rotateAxis);

    if (vtkMath::Dot(vpn, rotateAxis) > 0.0)
      {
      theta = -theta;
      }

    this->Internal->Transform->Translate(-centerOfRotation[0], -centerOfRotation[1], -centerOfRotation[2]);
    this->Internal->Transform->RotateWXYZ(theta, rotateAxis);
    this->Internal->Transform->Translate(centerOfRotation[0], centerOfRotation[1], centerOfRotation[2]);

    }
  else
    {

    double axis[3]; //axis of rotation
    double theta; //rotation angle
    double p1[3];
    double p2[3];

    vtkInteractorObserver::ComputeDisplayToWorld(this->Renderer,
                     this->LastEventPosition[0], this->LastEventPosition[1], 0.0, p1);

    vtkInteractorObserver::ComputeDisplayToWorld(this->Renderer, e[0], e[1], 0.0, p2);

    double v[3]; //vector of motion
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

    double l2 = (e[0]-this->LastEventPosition[0])*(e[0]-this->LastEventPosition[0])
               + (e[1]-this->LastEventPosition[1])*(e[1]-this->LastEventPosition[1]);
    theta = 360.0 * sqrt(l2/(size[0]*size[0]+size[1]*size[1]));


    this->Internal->Transform->Translate(-centerOfRotation[0], -centerOfRotation[1], -centerOfRotation[2]);
    this->Internal->Transform->RotateWXYZ(theta,axis);
    this->Internal->Transform->Translate(centerOfRotation[0], centerOfRotation[1], centerOfRotation[2]);
    }

  this->Internal->Transform->Modified();
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
  /*
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
  */


  for (size_t i = 0; i < this->Internal->Reps.size(); ++i)
    {
    if (this->Internal->Reps[i].PolyData.GetPointer() == dataset)
      {
      this->Internal->Reps[i].Actor->GetProperty()->SetAmbient(0.7);
      this->Internal->Reps[i].Actor->GetProperty()->SetAmbientColor(this->Internal->Reps[i].Actor->GetProperty()->GetColor());
      }
    else
      {
      this->Internal->Reps[i].Actor->GetProperty()->SetAmbient(0.0);
      }
    }

  for (size_t i = 0; i < this->Internal->Axes.size(); ++i)
    {
    if (this->Internal->Axes[i].PolyData.GetPointer() == dataset)
      {
      this->Internal->Axes[i].Actor->GetProperty()->SetAmbient(0.7);
      this->Internal->Axes[i].Actor->GetProperty()->SetAmbientColor(this->Internal->Axes[i].Actor->GetProperty()->GetColor());
      }
    else
      {
      this->Internal->Axes[i].Actor->GetProperty()->SetAmbient(0.0);
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

  this->RotateAxis = -1;
  this->TranslateAxis = -1;

  this->Internal->Transform->GetPosition(this->InteractionStartWorldPoint);

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
        this->TranslateAxis = i;
        this->RotateAxis = i;
        }
      }

    for (size_t i = 0; i < this->Internal->Axes.size(); ++i)
      {
      if (this->Internal->Axes[i].PolyData.GetPointer() == dataset)
        {
        this->InteractionState = vtkFrameWidgetRepresentation::Translating;
        this->TranslateAxis = i;
        this->RotateAxis = i;
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
void vtkFrameWidgetRepresentation::GetActors(vtkPropCollection* propCollection)
{
  if (!propCollection)
    {
    return;
    }

  for (size_t i = 0; i < this->Internal->Actors.size(); ++i)
    {
    propCollection->AddItem(this->Internal->Actors[i]);
    }
}

//----------------------------------------------------------------------------
void vtkFrameWidgetRepresentation::BuildRepresentation()
{
  if (this->GetMTime() > this->BuildTime)
    {
    this->BuildTime.Modified();
    this->Internal->RebuildActors(this->WorldSize, this->UseTubeFilter);
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
    if (this->Internal->Actors[i]->GetVisibility())
      {
      count += this->Internal->Actors[i]->RenderOpaqueGeometry(v);
      }
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
    if (this->Internal->Actors[i]->GetVisibility())
      {
      count += this->Internal->Actors[i]->RenderOverlay(v);
      }
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
    if (this->Internal->Actors[i]->GetVisibility())
      {
      count += this->Internal->Actors[i]->RenderTranslucentPolygonalGeometry(v);
      }
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
    if (this->Internal->Actors[i]->GetVisibility())
      {
      result |= this->Internal->Actors[i]->HasTranslucentPolygonalGeometry();
      }
    }

  return result;
}

//----------------------------------------------------------------------------
void vtkFrameWidgetRepresentation::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);

  os << indent << "Transform: " << this->Internal->Transform << "\n";
}
