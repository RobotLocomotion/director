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

  // delete pythonManager;
  // Allow a leak to avoid a segfault in the PythonQt cleanup destructor.
  // The segfault is fixed in upstream PythonQt, but we can't upgrade to
  // that version yet because of a compile issue on Ubuntu 12 + Qt 4.8.

  return result;
}
