#include <PythonQt.h>
#include <QApplication>
#include "ddPythonManager.h"

int main(int argc, char **argv)
{
  QApplication app(argc, argv);
  ddPythonManager* pythonManager = new ddPythonManager;
  pythonManager->setSysArgv(QApplication::instance()->arguments());
  PythonQt::self()->addVariable(PythonQt::self()->importModule("sys"), "executable", QCoreApplication::applicationFilePath());
  pythonManager->executeString("import director.drakevisualizer; director.drakevisualizer.main(globals())");

  // delete pythonManager;
  // Allow a leak to avoid a segfault in the PythonQt cleanup destructor.
  // The segfault is fixed in upstream PythonQt, but we can't upgrade to
  // that version yet because of a compile issue on Ubuntu 12 + Qt 4.8.

  return 0;
}
