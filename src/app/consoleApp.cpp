#include <QApplication>
#include "ddPythonManager.h"

int main(int argc, char **argv)
{
  QApplication app(argc, argv);
  ddPythonManager pythonManager;
  pythonManager.handleCommandLineArgs();
}
