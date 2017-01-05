#include <PythonQt.h>
#include <QApplication>
#include "ddPythonManager.h"

int main(int argc, char **argv)
{
  QApplication app(argc, argv);
  ddPythonManager* pythonManager = new ddPythonManager;
  pythonManager->setSysArgv(QApplication::instance()->arguments());
  PythonQt::self()->addVariable(PythonQt::self()->importModule("sys"), "executable", QCoreApplication::applicationFilePath());

  if (argc > 1 && strcmp(argv[1], "-v2") == 0) {
    pythonManager->executeString("import director.drakevisualizerapp2; director.drakevisualizerapp2.main(globals())");
  } else {
    pythonManager->executeString("import director.drakevisualizerapp1; director.drakevisualizerapp1.main(globals())");
  }


  // delete pythonManager;
  // Allow a leak to avoid a segfault in the PythonQt cleanup destructor.
  // The segfault is fixed in upstream PythonQt, but we can't upgrade to
  // that version yet because of a compile issue on Ubuntu 12 + Qt 4.8.

  return 0;
}
