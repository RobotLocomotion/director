#include "ddQVTKWidgetView.h"

#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkConeSource.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkAxesActor.h>

#include <QVTKWidget.h>
#include <QVBoxLayout>

//-----------------------------------------------------------------------------
class ddQVTKWidgetView::ddInternal
{
public:

  QVTKWidget* VTKWidget;

  vtkSmartPointer<vtkRenderer> Renderer;
  vtkSmartPointer<vtkRenderWindow> RenderWindow;

  vtkSmartPointer<vtkOrientationMarkerWidget> OrientationWidget;
};


//-----------------------------------------------------------------------------
ddQVTKWidgetView::ddQVTKWidgetView(QWidget* parent) : ddViewBase(parent)
{
  this->Internal = new ddInternal;

  QVBoxLayout* layout = new QVBoxLayout(this);
  layout->setMargin(0);
  this->Internal->VTKWidget = new QVTKWidget;
  layout->addWidget(this->Internal->VTKWidget);

  this->Internal->RenderWindow = vtkSmartPointer<vtkRenderWindow>::New();
  this->Internal->RenderWindow->SetMultiSamples(8);
  this->Internal->RenderWindow->StereoCapableWindowOn();
  this->Internal->RenderWindow->SetStereoTypeToRedBlue();
  this->Internal->RenderWindow->StereoRenderOff();
  this->Internal->RenderWindow->StereoUpdate();
  this->Internal->VTKWidget->SetRenderWindow(this->Internal->RenderWindow);

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
void ddQVTKWidgetView::render()
{
  this->Internal->RenderWindow->Render();
  this->update();
}

//-----------------------------------------------------------------------------
void ddQVTKWidgetView::resetCamera()
{
  this->renderer()->ResetCamera();
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
