#include <Python.h>
#include <PythonQt.h>
#include <QApplication>
#include "ddPythonManager.h"
#include "QVTKOpenGLInit.h"

#include "py3main.h"

int main(int argc, char **argv)
{
  QVTKOpenGLInit init;
  QApplication app(argc, argv);
  ddPythonManager* pythonManager = new ddPythonManager;
  PythonQt::self()->addVariable(PythonQt::self()->importModule("sys"), "executable", QCoreApplication::applicationFilePath());
#if PY_MAJOR_VERSION >= 3
  int result = Py3_Main(argc, argv);
#else
  int result = Py_Main(argc, argv);
#endif

  delete pythonManager;
  return result;
}
