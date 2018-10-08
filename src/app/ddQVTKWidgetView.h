#ifndef __ddQVTKWidgetView_h
#define __ddQVTKWidgetView_h

#include "ddViewBase.h"
#include "ddAppConfigure.h"
#include "ddQVTKOpenGLWidgetConfigure.h"


class vtkCamera;
class vtkOrientationMarkerWidget;
class vtkRenderer;
class vtkRenderWindow;
class vtkLightKit;

class DD_APP_EXPORT ddQVTKWidgetView : public ddViewBase
{
    Q_OBJECT

public:

  ddQVTKWidgetView(QWidget* parent=0);
  virtual ~ddQVTKWidgetView();

  vtkRenderWindow* renderWindow() const;
  vtkRenderer* renderer() const;
  vtkRenderer* backgroundRenderer() const;
  vtkCamera* camera() const;
  vtkLightKit* lightKit() const;

  QList<double> lastTDxMotion() const;

  QVTKOpenGLWidget* vtkWidget() const;
  vtkOrientationMarkerWidget* orientationMarkerWidget() const;

  QTimer* renderTimer() const;

  void installImageInteractor();

  void addCustomBounds(const QList<double>& bounds);

  void setLightKitEnabled(bool enabled);

  double getAverageFramesPerSecond();

  static void setAntiAliasing(bool enabled);

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
  void onRenderTimer();

protected:

  void setupOrientationMarker();

  void addCone();

  class ddInternal;
  ddInternal* Internal;

  static bool antiAliasingEnabled;

  Q_DISABLE_COPY(ddQVTKWidgetView);
};

#endif
