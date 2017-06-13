// Qt includes
#include <QApplication>

// director includes
#include <PythonQt.h>
#include "ddPythonManager.h"
#include "QVTKOpenGLInit.h"

int main(int argc, char **argv)
{
  QVTKOpenGLInit init;
  QApplication app(argc, argv);
  ddPythonManager* pythonManager = new ddPythonManager;
  pythonManager->setSysArgv(QApplication::instance()->arguments());
  PythonQt::self()->addVariable(PythonQt::self()->importModule("sys"), "executable", QCoreApplication::applicationFilePath());

  PythonQt::self()->importModule("director");
  pythonManager->executeString("import director.drakevisualizerapp; director.drakevisualizerapp.main(globals())");

  delete pythonManager;
  return 0;
}
