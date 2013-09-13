#include "ddMainWindow.h"
#include "ddMacros.h"
#include "ddPythonManager.h"
#include "ddViewManager.h"

#include "ui_ddMainWindow.h"

#include <QApplication>
#include <QTimer>
#include <QShortcut>

#include <cstdio>


//-----------------------------------------------------------------------------
class ddMainWindow::ddInternal : public Ui::ddMainWindow
{
public:

  ddViewManager* ViewManager;

  ddPythonManager* PythonManager;
};


//-----------------------------------------------------------------------------
ddMainWindow::ddMainWindow()
{
  this->Internal = new ddInternal;
  this->Internal->setupUi(this);

  this->Internal->ViewManager = new ddViewManager;
  this->setCentralWidget(this->Internal->ViewManager);

  this->setWindowTitle("Drake Designer");
  this->connect(this->Internal->ActionQuit, SIGNAL(triggered()), QApplication::instance(), SLOT(quit()));

  this->setupPython();

  QTimer::singleShot(0, this, SLOT(startup()));
}

//-----------------------------------------------------------------------------
ddMainWindow::~ddMainWindow()
{
  delete this->Internal;
}

//-----------------------------------------------------------------------------
void ddMainWindow::handleCommandLineArgs()
{
  QStringList args = QApplication::instance()->arguments();
  ddNotUsed(args);
}

//-----------------------------------------------------------------------------
void ddMainWindow::startup()
{
  this->handleCommandLineArgs();
}

//-----------------------------------------------------------------------------
void ddMainWindow::setupPython()
{
  this->Internal->PythonManager = new ddPythonManager(this);
  this->Internal->PythonManager->addObjectToPythonMain("_mainWindow", this);
  this->Internal->PythonManager->setupConsole(this);
}
