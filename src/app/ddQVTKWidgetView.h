#ifndef __ddQVTKWidgetView_h
#define __ddQVTKWidgetView_h

#include "ddViewBase.h"

class vtkCamera;
class vtkRenderer;
class vtkRenderWindow;
class QVTKWidget;

class ddQVTKWidgetView : public ddViewBase
{
    Q_OBJECT

public:

  ddQVTKWidgetView(QWidget* parent=0);
  virtual ~ddQVTKWidgetView();

  vtkRenderWindow* renderWindow() const;
  vtkRenderer* renderer() const;
  vtkCamera* camera() const;

  QList<double> lastTDxMotion() const;

  QVTKWidget* vtkWidget() const;

  void installImageInteractor();

public slots:

  void render();
  void forceRender();
  void resetCamera();

  void setActorManipulationStyle();
  void setCameraManipulationStyle();

protected:

  void setupOrientationMarker();

  void addCone();

  class ddInternal;
  ddInternal* Internal;

  Q_DISABLE_COPY(ddQVTKWidgetView);
};

#endif
