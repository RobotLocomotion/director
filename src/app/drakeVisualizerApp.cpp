#include <PythonQt.h>
#include <QApplication>
#include "ddPythonManager.h"

int main(int argc, char **argv)
{
  QApplication app(argc, argv);
  ddPythonManager* pythonManager = new ddPythonManager;
  PythonQt::self()->addVariable(PythonQt::self()->importModule("sys"), "executable", QCoreApplication::applicationFilePath());
  pythonManager->executeString("import ddapp.drakevisualizer; ddapp.drakevisualizer.main(globals())");

  // delete pythonManager;
  // Allow a leak to avoid a segfault in the PythonQt cleanup destructor.
  // The segfault is fixed in upstream PythonQt, but we can't upgrade to
  // that version yet because of a compile issue on Ubuntu 12 + Qt 4.8.

  return 0;
}
