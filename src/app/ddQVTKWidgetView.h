#ifndef __ddQVTKWidgetView_h
#define __ddQVTKWidgetView_h

#include "ddViewBase.h"

class vtkCamera;
class vtkOrientationMarkerWidget;
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
  vtkRenderer* backgroundRenderer() const;
  vtkCamera* camera() const;

  QList<double> lastTDxMotion() const;

  QVTKWidget* vtkWidget() const;
  vtkOrientationMarkerWidget* orientationMarkerWidget() const;

  void installImageInteractor();

  void addCustomBounds(const QList<double>& bounds);

signals:

  void computeBoundsRequest(ddQVTKWidgetView* view);

public slots:

  void render();
  void forceRender();
  void resetCamera();

  void setActorManipulationStyle();
  void setCameraManipulationStyle();

protected slots:

   void onStartRender();
   void onEndRender();

protected:

  void setupOrientationMarker();

  void addCone();

  class ddInternal;
  ddInternal* Internal;

  Q_DISABLE_COPY(ddQVTKWidgetView);
};

#endif
