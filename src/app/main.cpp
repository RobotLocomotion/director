#include <QApplication>
#include <QVTKApplication.h>

#include "ddMainWindow.h"
#include "ddPythonManager.h"

int main(int argc, char **argv)
{
  QVTKApplication app(argc, argv);

  ddPythonManager pythonManager;

  ddMainWindow window;
  window.setPythonManager(&pythonManager);
  window.resize(1800, 1000);
  window.show();

  return app.exec();
}
