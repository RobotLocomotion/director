#include <QApplication>
#include "ddMainWindow.h"
#include "ddPythonManager.h"

#define USE_TDX 0
#if USE_TDX
  #include <QVTKApplication.h>
#endif

int main(int argc, char **argv)
{
#if USE_TDX
  QVTKApplication app(argc, argv);
#else
  QApplication app(argc, argv);
#endif

  ddPythonManager* pythonManager = new ddPythonManager;

  ddMainWindow* window = new ddMainWindow;
  window->setPythonManager(pythonManager);
  window->resize(1800, 1000);
  window->show();

  int result = app.exec();

  delete window;

#ifdef Q_OS_MAC
  // On MacOSX, there is a crash during PythonQt's finalization cleanup
  // while destructing class metadata objects.
#else
  delete pythonManager;
#endif

  return result;
}
