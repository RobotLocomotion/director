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
  // crashes on Ubuntu 14.04 with Qt version 4.8.6
  // this is a bug in PythonQt's finalize I think, to be investigated
  //delete pythonManager;
#endif

  return result;
}
