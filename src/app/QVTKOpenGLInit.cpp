#include "QVTKOpenGLInit.h"

// Qt includes
#include <QtGlobal>

// VTK includes
#include <vtkVersionMacros.h>
#include <vtkRenderingOpenGLConfigure.h>

#if QT_VERSION >= QT_VERSION_CHECK(5, 4, 0) && VTK_MAJOR_VERSION >= 8
#  ifdef VTK_OPENGL2
  #include <QSurfaceFormat>
  #include <QVTKOpenGLWidget.h>
#  endif
#endif

QVTKOpenGLInit::QVTKOpenGLInit()
{
#if QT_VERSION >= QT_VERSION_CHECK(5, 4, 0) && VTK_MAJOR_VERSION >= 8
#  ifdef VTK_OPENGL2
  // Set the default surface format for the OpenGL view
  QSurfaceFormat::setDefaultFormat(QVTKOpenGLWidget::defaultFormat());
#  endif
#endif
}
