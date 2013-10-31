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

  ddMainWindow window;
  window.setPythonManager(pythonManager);
  window.resize(1800, 1000);
  window.show();

  return app.exec();
}
