#include <Python.h>
#include <PythonQt.h>
#include <QApplication>
#include "ddPythonManager.h"

int main(int argc, char **argv)
{
  QApplication app(argc, argv);
  ddPythonManager* pythonManager = new ddPythonManager;
  PythonQt::self()->addVariable(PythonQt::self()->importModule("sys"), "executable", QCoreApplication::applicationFilePath());
  int result = Py_Main(argc, argv);

#ifdef Q_OS_MAC
  // On MacOSX, there is a crash during PythonQt's finalization cleanup
  // while destructing class metadata objects.
#else
  delete pythonManager;
#endif

  return result;
}
