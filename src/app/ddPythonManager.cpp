#include "ddPythonManager.h"

#include "ddPythonQtWrapperFactory.h"
#include "ddPythonQtDecorators.h"

#include <ctkPythonConsole.h>

#include <QApplication>
#include <QShortcut>

//-----------------------------------------------------------------------------
class ddPythonManager::ddInternal
{
public:

  ctkPythonConsole* Console;
};


//-----------------------------------------------------------------------------
ddPythonManager::ddPythonManager(QObject* parent) : ctkAbstractPythonManager(parent)
{
  this->Internal = new ddInternal;

  ctkPythonConsole* console = new ctkPythonConsole;


  console->setWindowFlags(Qt::Dialog | Qt::WindowStaysOnTopHint);
  console->initialize(this);
  console->setAttribute(Qt::WA_QuitOnClose, true);
  console->resize(600, 280);
  console->setProperty("isInteractive", true);
  this->Internal->Console = console;
  this->setupConsoleShortcuts();
  this->addObjectToPythonMain("_console", console);

  this->executeString("import sys; sys.path += ['/source/paraview/build/VTK/Wrapping/Python', '/source/paraview/build/lib']");
  this->executeString("def quit(): _mainWindow.close()");
  this->executeString("exit = quit");
  //this->executeFile("/source/drc/drc-trunk/software/motion_estimate/signal_scope/src/signal_scope/numpy_test.py");
}

//-----------------------------------------------------------------------------
ddPythonManager::~ddPythonManager()
{
  delete this->Internal->Console;
  delete this->Internal;
}

//-----------------------------------------------------------------------------
void ddPythonManager::preInitialization()
{
  this->addWrapperFactory(new ddPythonQtWrapperFactory);
  this->registerPythonQtDecorator(new ddPythonQtDecorators);
}

//-----------------------------------------------------------------------------
void ddPythonManager::showConsole()
{
  this->Internal->Console->show();
}

//-----------------------------------------------------------------------------
void ddPythonManager::setupConsole(QWidget* parent)
{
  this->Internal->Console->setParent(parent);
  this->Internal->Console->setWindowFlags(Qt::Dialog | Qt::WindowStaysOnTopHint);
}

//-----------------------------------------------------------------------------
void ddPythonManager::setupConsoleShortcuts()
{
  ctkPythonConsole* console = this->Internal->Console;
  this->connect(new QShortcut(QKeySequence("Ctrl+W"), console), SIGNAL(activated()), console, SLOT(close()));

  QString closeShortcut = "Ctrl+D";
  #ifdef Q_OS_DARWIN
  closeShortcut = "Meta+D";
  #endif
  this->connect(new QShortcut(QKeySequence(closeShortcut), console), SIGNAL(activated()), console, SLOT(close()));
}
