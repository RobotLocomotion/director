// Qt includes
#include <QApplication>
#include <QSurfaceFormat>

// director includes
#include <PythonQt.h>
#include "ddPythonManager.h"

// VTK includes
#if QT_VERSION >= QT_VERSION_CHECK(5, 4, 0)
  #include <QVTKOpenGLWidget.h>
#endif

int main(int argc, char **argv)
{
#if QT_VERSION >= QT_VERSION_CHECK(5, 4, 0)
  // Set the default surface format for the OpenGL view
  QSurfaceFormat::setDefaultFormat(QVTKOpenGLWidget::defaultFormat());
#endif
  QApplication app(argc, argv);
  ddPythonManager* pythonManager = new ddPythonManager;
  pythonManager->setSysArgv(QApplication::instance()->arguments());
  PythonQt::self()->addVariable(PythonQt::self()->importModule("sys"), "executable", QCoreApplication::applicationFilePath());

  PythonQt::self()->importModule("director");
  pythonManager->executeString("import director.drakevisualizerapp; director.drakevisualizerapp.main(globals())");

  // delete pythonManager;
  // Allow a leak to avoid a segfault in the PythonQt cleanup destructor.
  // The segfault is fixed in upstream PythonQt, but we can't upgrade to
  // that version yet because of a compile issue on Ubuntu 12 + Qt 4.8.

  return 0;
}
