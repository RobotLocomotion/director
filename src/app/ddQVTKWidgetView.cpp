#include "ddQVTKWidgetView.h"

#include "vtkTDxInteractorStyleCallback.h"
#include "vtkSimpleActorInteractor.h"

#include <vtkBoundingBox.h>
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkInteractorStyle.h>
#include <vtkObjectFactory.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkConeSource.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkInteractorStyleRubberBand3D.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkAxesActor.h>

#include <QVTKWidget.h>
#include <QVBoxLayout>
#include <QTimer>

//-----------------------------------------------------------------------------
class vtkCustomRubberBandStyle : public vtkInteractorStyleRubberBand3D
{
public:

  static vtkCustomRubberBandStyle *New();
  vtkTypeMacro(vtkCustomRubberBandStyle, vtkInteractorStyleRubberBand3D);

  virtual void OnRightButtonDown()
  {
    if(this->Interaction == NONE)
      {
      this->Interaction = ZOOMING;
      this->FindPokedRenderer(
        this->Interactor->GetEventPosition()[0], 
        this->Interactor->GetEventPosition()[1]);
      this->InvokeEvent(vtkCommand::StartInteractionEvent);
      }
  }

};

vtkStandardNewMacro(vtkCustomRubberBandStyle);


//-----------------------------------------------------------------------------
class ddQVTKWidgetView::ddInternal
{
public:

  ddInternal()
  {
    this->RenderPending = false;
  }

  QVTKWidget* VTKWidget;

  vtkSmartPointer<vtkRenderer> Renderer;
  vtkSmartPointer<vtkRenderWindow> RenderWindow;

  vtkSmartPointer<vtkOrientationMarkerWidget> OrientationWidget;

  vtkSmartPointer<vtkTDxInteractorStyleCallback> TDxInteractor;

  QList<QList<double> > CustomBounds;

  bool RenderPending;
};


//-----------------------------------------------------------------------------
ddQVTKWidgetView::ddQVTKWidgetView(QWidget* parent) : ddViewBase(parent)
{
  this->Internal = new ddInternal;

  QVBoxLayout* layout = new QVBoxLayout(this);
  layout->setMargin(0);
  this->Internal->VTKWidget = new QVTKWidget;
  layout->addWidget(this->Internal->VTKWidget);

  this->Internal->VTKWidget->SetUseTDx(true);

  this->Internal->RenderWindow = vtkSmartPointer<vtkRenderWindow>::New();
  this->Internal->RenderWindow->SetMultiSamples(8);
  this->Internal->RenderWindow->StereoCapableWindowOn();
  this->Internal->RenderWindow->SetStereoTypeToRedBlue();
  this->Internal->RenderWindow->StereoRenderOff();
  this->Internal->RenderWindow->StereoUpdate();
  this->Internal->VTKWidget->SetRenderWindow(this->Internal->RenderWindow);

  this->Internal->TDxInteractor = vtkSmartPointer<vtkTDxInteractorStyleCallback>::New();
  vtkInteractorStyle::SafeDownCast(this->Internal->RenderWindow->GetInteractor()->GetInteractorStyle())->SetTDxStyle(this->Internal->TDxInteractor);

  this->Internal->Renderer = vtkSmartPointer<vtkRenderer>::New();
  this->Internal->RenderWindow->AddRenderer(this->Internal->Renderer);

  //vtkMapper::SetResolveCoincidentTopologyToPolygonOffset();

  this->Internal->Renderer->GradientBackgroundOn();
  this->Internal->Renderer->SetBackground(0.0, 0.0, 0.0);
  this->Internal->Renderer->SetBackground2(0.3, 0.3, 0.3);

  this->setupOrientationMarker();

  this->Internal->Renderer->ResetCamera();
}

//-----------------------------------------------------------------------------
ddQVTKWidgetView::~ddQVTKWidgetView()
{
  delete this->Internal;
}

//-----------------------------------------------------------------------------
vtkCamera* ddQVTKWidgetView::camera() const
{
  return this->Internal->Renderer->GetActiveCamera();
}

//-----------------------------------------------------------------------------
 void ddQVTKWidgetView::installImageInteractor()
{
  this->renderWindow()->GetInteractor()->SetInteractorStyle(vtkCustomRubberBandStyle::New());
}

//-----------------------------------------------------------------------------
vtkRenderWindow* ddQVTKWidgetView::renderWindow() const
{
  return this->Internal->VTKWidget->GetRenderWindow();
}

//-----------------------------------------------------------------------------
vtkRenderer* ddQVTKWidgetView::renderer() const
{
  return this->Internal->Renderer;
}

//-----------------------------------------------------------------------------
vtkRenderer* ddQVTKWidgetView::backgroundRenderer() const
{
  return this->Internal->Renderer;
}

//-----------------------------------------------------------------------------
QVTKWidget* ddQVTKWidgetView::vtkWidget() const
{
  return this->Internal->VTKWidget;
}

//-----------------------------------------------------------------------------
void ddQVTKWidgetView::render()
{
  if (!this->Internal->RenderPending)
  {
    this->Internal->RenderPending = true;
    QTimer::singleShot(0, this, SLOT(forceRender()));
  }
}

//-----------------------------------------------------------------------------
void ddQVTKWidgetView::forceRender()
{
  this->Internal->Renderer->ResetCameraClippingRange();
  this->Internal->RenderWindow->Render();
  this->update();
  this->Internal->RenderPending = false;
}

//-----------------------------------------------------------------------------
void ddQVTKWidgetView::addCustomBounds(const QList<double>& bounds)
{
  this->Internal->CustomBounds.append(bounds);
}

//-----------------------------------------------------------------------------
void ddQVTKWidgetView::resetCamera()
{
  this->Internal->CustomBounds.clear();
  emit this->computeBoundsRequest(this);

  if (this->Internal->CustomBounds.size())
  {
    vtkBoundingBox bbox;
    foreach (const QList<double>& b, this->Internal->CustomBounds)
    {
      double bounds[6] = {b[0], b[1], b[2], b[3], b[4], b[5]};
      bbox.AddBounds(bounds);
    }
    double bounds[6];
    bbox.GetBounds(bounds);
    this->renderer()->ResetCamera(bounds);
  }
  else
  {
    this->renderer()->ResetCamera();
  }
}

//-----------------------------------------------------------------------------
QList<double> ddQVTKWidgetView::lastTDxMotion() const
{
  double motionInfo[7];
  this->Internal->TDxInteractor->GetTranslation(motionInfo);
  this->Internal->TDxInteractor->GetAngleAxis(motionInfo+3);

  QList<double> motionInfoList;
  for (int i = 0; i < 7; ++i)
  {
    motionInfoList << motionInfo[i];
  }

  return motionInfoList;
}

//-----------------------------------------------------------------------------
void ddQVTKWidgetView::setActorManipulationStyle()
{
  this->renderWindow()->GetInteractor()->SetInteractorStyle(vtkSmartPointer<vtkSimpleActorInteractor>::New());
}

//-----------------------------------------------------------------------------
void ddQVTKWidgetView::setCameraManipulationStyle()
{
  this->renderWindow()->GetInteractor()->SetInteractorStyle(vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New());
}

//-----------------------------------------------------------------------------
void ddQVTKWidgetView::setupOrientationMarker()
{
  this->renderWindow()->GetInteractor()->Disable();
  vtkSmartPointer<vtkAxesActor> axesActor = vtkSmartPointer<vtkAxesActor>::New();
  vtkSmartPointer<vtkOrientationMarkerWidget> widget = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
  widget->SetOutlineColor(1.0, 1.0, 1.0);
  widget->SetOrientationMarker(axesActor);
  widget->SetInteractor(this->renderWindow()->GetInteractor());
  widget->SetViewport(0.0, 0.0, 0.2, 0.2);
  widget->SetEnabled(1);
  widget->InteractiveOff();
  this->Internal->OrientationWidget = widget;
  this->renderWindow()->GetInteractor()->Enable();
}

//-----------------------------------------------------------------------------
void ddQVTKWidgetView::addCone()
{
  vtkSmartPointer<vtkConeSource> cone = vtkSmartPointer<vtkConeSource>::New();
  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  cone->SetResolution( 8 );
  mapper->AddInputConnection(cone->GetOutputPort());
  actor->SetMapper(mapper);
  this->Internal->Renderer->AddActor(actor);
}
