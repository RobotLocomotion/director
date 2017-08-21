#include "ddPythonApp.h"

#include <QApplication>
#include <PythonQt.h>
#include "ddPythonManager.h"
#include "QVTKOpenGLInit.h"

//-----------------------------------------------------------------------------
class ddPythonApp::ddInternal
{
public:
  bool ownsApp;
  QApplication* app;
  ddPythonManager* pythonManager;
};


//-----------------------------------------------------------------------------
ddPythonApp::ddPythonApp()
{
  this->Internal = new ddInternal;
  this->Internal->ownsApp = false;
}

//-----------------------------------------------------------------------------
ddPythonApp::~ddPythonApp()
{
  delete this->Internal->pythonManager;
  if (this->Internal->ownsApp)
  {
    delete this->Internal->app;
  }
  delete this->Internal;
}

//-----------------------------------------------------------------------------
bool ddPythonApp::init()
{
  static char arg0[] = "program_name_not_set";
  static char* argv[] = {&arg0[0], NULL};
  static int argc = static_cast<int>((sizeof(argv) / sizeof(argv[0])) - 1);
  return this->init(argc, argv);
}

//-----------------------------------------------------------------------------
bool ddPythonApp::init(int& argc, char **argv)
{
  QVTKOpenGLInit init;
  this->Internal->ownsApp = true;
  return this->init(new QApplication(argc, argv));
}

//-----------------------------------------------------------------------------
bool ddPythonApp::init(QApplication* app)
{
  this->Internal->app = app;
  this->Internal->pythonManager = new ddPythonManager;
  this->Internal->pythonManager->setSysArgv(this->Internal->app->arguments());
  PythonQt::self()->addVariable(PythonQt::self()->importModule("sys"), "executable", QCoreApplication::applicationFilePath());
  PythonQtObjectPtr mod = PythonQt::self()->importModule("director");
  if (mod)
  {
    this->Internal->pythonManager->executeString(
      "import director.mainwindowapp;"
      "fields = director.mainwindowapp.construct(globals());"
      "globals().update(**dict(fields));"
      "fields._fields.remove('globalsDict');");
    return true;
  }
  return false;
}

//-----------------------------------------------------------------------------
void ddPythonApp::start()
{
  this->Internal->pythonManager->executeString("fields.app.start()");
}

//-----------------------------------------------------------------------------
void ddPythonApp::runPythonString(const std::string& str)
{
  this->Internal->pythonManager->executeString(str.c_str());
}
