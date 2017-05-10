#ifndef __ddQVTKWidgetView_h
#define __ddQVTKWidgetView_h

#include "ddViewBase.h"
#include "ddAppConfigure.h"


class vtkCamera;
class vtkOrientationMarkerWidget;
class vtkRenderer;
class vtkRenderWindow;
class vtkLightKit;
#if QT_VERSION >= QT_VERSION_CHECK(5, 4, 0)
  class QVTKOpenGLWidget;
#else
  class QVTKWidget;
  typedef QVTKWidget QVTKOpenGLWidget;
#endif

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

  void installImageInteractor();

  void addCustomBounds(const QList<double>& bounds);

  void setLightKitEnabled(bool enabled);

  double getAverageFramesPerSecond();

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

  Q_DISABLE_COPY(ddQVTKWidgetView);
};

#endif
