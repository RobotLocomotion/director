#include <pybind11/pybind11.h>
#include <iostream>

#include <QApplication>
#include "ddPythonManager.h"
#include "QVTKOpenGLInit.h"

#include <iostream>

namespace {
  QApplication* application = nullptr;
  ddPythonManager* pythonManager = nullptr;
}


void init() {
  if (application) {
    return;
  }

  static char arg0[] = "";
  static char* argv[] = {&arg0[0], NULL};
  static int argc = static_cast<int>((sizeof(argv) / sizeof(argv[0])) - 1);

  QVTKOpenGLInit init;
  application = new QApplication(argc, argv);
  pythonManager = new ddPythonManager;
}

void deinit() {
  delete pythonManager;
  delete application;
}


PYBIND11_MODULE(_pythonqt, m) {
    m.doc() = "PythonQt initializer";
    m.def("init", &init, "Initialize QApplication and PythonQt");
    m.def("deinit", &deinit, "Deinitialize QApplication and PythonQt");
}
