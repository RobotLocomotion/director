#include "ddQVTKWidgetView.h"
#include "ddFPSCounter.h"

#include "vtkTDxInteractorStyleCallback.h"
#include "vtkSimpleActorInteractor.h"

#include <vtkBoundingBox.h>
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkInteractorStyle.h>
#include <vtkLight.h>
#include <vtkLightKit.h>
#include <vtkLightCollection.h>
#include <vtkObjectFactory.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkConeSource.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkInteractorStyleRubberBand3D.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkAxesActor.h>
#include <vtkEventQtSlotConnect.h>
#include <vtkCaptionActor2D.h>
#include <vtkTextProperty.h>

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
    this->Connector = vtkSmartPointer<vtkEventQtSlotConnect>::New();
    this->RenderTimer.setSingleShot(false);

    int timerFramesPerSeconds = 60;
    this->RenderTimer.setInterval(1000/timerFramesPerSeconds);
  }

  QVTKWidget* VTKWidget;

  vtkSmartPointer<vtkRenderer> Renderer;
  vtkSmartPointer<vtkRenderer> RendererBase;
  vtkSmartPointer<vtkRenderWindow> RenderWindow;
  vtkSmartPointer<vtkLightKit> LightKit;

  vtkSmartPointer<vtkOrientationMarkerWidget> OrientationWidget;

  vtkSmartPointer<vtkTDxInteractorStyleCallback> TDxInteractor;

  vtkSmartPointer<vtkEventQtSlotConnect> Connector;

  QList<QList<double> > CustomBounds;

  bool RenderPending;

  ddFPSCounter FPSCounter;
  QTimer RenderTimer;
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

  this->Internal->LightKit = vtkSmartPointer<vtkLightKit>::New();
  this->Internal->LightKit->SetKeyLightWarmth(0.5);
  this->Internal->LightKit->SetFillLightWarmth(0.5);

  this->Internal->TDxInteractor = vtkSmartPointer<vtkTDxInteractorStyleCallback>::New();
  vtkInteractorStyle::SafeDownCast(this->Internal->RenderWindow->GetInteractor()->GetInteractorStyle())->SetTDxStyle(this->Internal->TDxInteractor);

  //this->Internal->RenderWindow->SetNumberOfLayers(2);

  //this->Internal->RendererBase = vtkSmartPointer<vtkRenderer>::New();
  //this->Internal->RenderWindow->AddRenderer(this->Internal->RendererBase);

  this->Internal->Renderer = vtkSmartPointer<vtkRenderer>::New();
  //this->Internal->Renderer->SetLayer(1);
  this->Internal->RenderWindow->AddRenderer(this->Internal->Renderer);




  vtkMapper::SetResolveCoincidentTopologyToPolygonOffset();

  this->Internal->Renderer->GradientBackgroundOn();
  this->Internal->Renderer->SetBackground(0.0, 0.0, 0.0);
  this->Internal->Renderer->SetBackground2(0.3, 0.3, 0.3);

  this->Internal->Connector->Connect(this->Internal->Renderer, vtkCommand::StartEvent, this, SLOT(onStartRender()));
  this->Internal->Connector->Connect(this->Internal->Renderer, vtkCommand::EndEvent, this, SLOT(onEndRender()));

  this->setupOrientationMarker();

  this->Internal->Renderer->ResetCamera();

  this->connect(&this->Internal->RenderTimer, SIGNAL(timeout()), SLOT(onRenderTimer()));
  this->Internal->RenderTimer.start();
  this->setLightKitEnabled(true);
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
vtkLightKit* ddQVTKWidgetView::lightKit() const
{
  return this->Internal->LightKit;
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
  }
}

//-----------------------------------------------------------------------------
void ddQVTKWidgetView::forceRender()
{
  this->Internal->Renderer->ResetCameraClippingRange();
  this->Internal->RenderWindow->Render();
}

//-----------------------------------------------------------------------------
void ddQVTKWidgetView::onStartRender()
{
  this->Internal->RenderPending = false;
}

//-----------------------------------------------------------------------------
void ddQVTKWidgetView::onEndRender()
{
  this->Internal->FPSCounter.update();
  //printf("end render: %.2f fps\n", this->Internal->FPSCounter.averageFPS());
}

//-----------------------------------------------------------------------------
void ddQVTKWidgetView::onRenderTimer()
{
  if (this->Internal->RenderPending)
  {
    this->forceRender();
  }
}

//-----------------------------------------------------------------------------
void ddQVTKWidgetView::addCustomBounds(const QList<double>& bounds)
{
  this->Internal->CustomBounds.append(bounds);
}

//-----------------------------------------------------------------------------
void ddQVTKWidgetView::setLightKitEnabled(bool enabled)
{
  this->renderer()->RemoveAllLights();
  if (enabled)
  {
    this->Internal->LightKit->AddLightsToRenderer(this->renderer());
  }
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

  this->Internal->Renderer->ResetCameraClippingRange();
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

namespace {
void SetTextProperty(vtkTextProperty* prop)
{
  prop->ShadowOff();
  prop->BoldOff();
  prop->ItalicOff();
  //prop->SetColor(0,0,0);
}
}

//-----------------------------------------------------------------------------
vtkOrientationMarkerWidget* ddQVTKWidgetView::orientationMarkerWidget() const
{
  return this->Internal->OrientationWidget;
}

//-----------------------------------------------------------------------------
void ddQVTKWidgetView::setupOrientationMarker()
{
  this->renderWindow()->GetInteractor()->Disable();
  vtkSmartPointer<vtkAxesActor> axesActor = vtkSmartPointer<vtkAxesActor>::New();
  SetTextProperty(axesActor->GetXAxisCaptionActor2D()->GetCaptionTextProperty());
  SetTextProperty(axesActor->GetYAxisCaptionActor2D()->GetCaptionTextProperty());
  SetTextProperty(axesActor->GetZAxisCaptionActor2D()->GetCaptionTextProperty());

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
