#include "ddMainWindow.h"

#include "ddMacros.h"
#include "ddPythonManager.h"
#include "ddViewManager.h"
#include "ddPropertiesPanel.h"
#include "ddViewMenu.h"

#include "ui_ddMainWindow.h"

#include <QApplication>
#include <QTimer>
#include <QShortcut>
#include <QStatusBar>
#include <QLabel>
#include <QPointer>

#include <cstdio>


//-----------------------------------------------------------------------------
class ddMainWindow::ddInternal : public Ui::ddMainWindow
{
public:

  ddViewManager* ViewManager;
  ddPropertiesPanel* PropertiesPanel;
  QPointer<ddPythonManager> PythonManager;

  ddViewMenu* ViewMenuManager;
};


//-----------------------------------------------------------------------------
ddMainWindow::ddMainWindow()
{
  this->Internal = new ddInternal;
  this->Internal->setupUi(this);

  this->Internal->ViewManager = new ddViewManager;
  this->Internal->PropertiesPanel = new ddPropertiesPanel;

  this->setCentralWidget(this->Internal->ViewManager);
  this->Internal->PropertiesDock->setWidget(this->Internal->PropertiesPanel);

  this->setWindowTitle("Drake Designer");
  this->connect(this->Internal->ActionQuit, SIGNAL(triggered()), QApplication::instance(), SLOT(quit()));

  this->Internal->OutputConsoleDock->hide();
  this->connect(this->Internal->ActionMatlabConsole, SIGNAL(triggered()), this, SLOT(toggleOutputConsoleVisibility()));
  this->connect(this->Internal->ActionResetCamera, SIGNAL(triggered()), this, SIGNAL(resetCamera()));
  this->connect(this->Internal->ActionToggleStereoRender, SIGNAL(triggered()), this, SIGNAL(toggleStereoRender()));
  this->connect(this->Internal->ActionToggleCameraTerrainMode, SIGNAL(triggered()), this, SIGNAL(toggleCameraTerrainMode()));

  QTimer::singleShot(0, this, SLOT(startup()));

  this->setCorner(Qt::TopLeftCorner, Qt::LeftDockWidgetArea);
  this->setCorner(Qt::TopRightCorner, Qt::RightDockWidgetArea);
  this->setCorner(Qt::BottomLeftCorner, Qt::LeftDockWidgetArea);
  this->setCorner(Qt::BottomRightCorner, Qt::RightDockWidgetArea);

  //QLabel* logoLabel = new QLabel();
  //logoLabel->setPixmap(QPixmap(":/images/drake_logo.png").scaled(QSize(32,32),  Qt::KeepAspectRatio));
  //logoLabel->setScaledContents(true);
  //this->statusBar()->addPermanentWidget(logoLabel);

  this->setupViewMenu();
}

//-----------------------------------------------------------------------------
ddMainWindow::~ddMainWindow()
{
  delete this->Internal;
}

//-----------------------------------------------------------------------------
ddViewManager* ddMainWindow::viewManager() const
{
  return this->Internal->ViewManager;
}

//-----------------------------------------------------------------------------
ddPropertiesPanel* ddMainWindow::propertiesPanel() const
{
  return this->Internal->PropertiesPanel;
}

//-----------------------------------------------------------------------------
QToolBar* ddMainWindow::toolBar() const
{
  return this->Internal->MainToolBar;
}

//-----------------------------------------------------------------------------
QTreeWidget* ddMainWindow::objectTree() const
{
  return this->Internal->ObjectTree;
}

//-----------------------------------------------------------------------------
QTextEdit* ddMainWindow::outputConsole() const
{
  return this->Internal->OutputConsole;
}

//-----------------------------------------------------------------------------
void ddMainWindow::toggleOutputConsoleVisibility()
{
  if (this->Internal->OutputConsoleDock->isHidden())
  {
    this->Internal->OutputConsoleDock->show();
  }
  else
  {
    this->Internal->OutputConsoleDock->hide();
  }
}

//-----------------------------------------------------------------------------
void ddMainWindow::addWidgetToViewMenu(QWidget* widget)
{
  if (!widget)
  {
    return;
  }

  this->Internal->ViewMenuManager->addWidget(widget, widget->windowTitle());
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

  this->setupPython();
  QString startupScript = this->Internal->PythonManager->appSitePackagesDir() + "/ddapp/startup.py";
  this->Internal->PythonManager->executeFile(startupScript);
}

//----------------------------------------------------------------------------
void ddMainWindow::setupViewMenu()
{
  ddViewMenu* viewMenu = new ddViewMenu(*this->Internal->ViewMenu, this);
  this->Internal->ViewMenuManager = viewMenu;
  ddViewMenu* toolbarMenu = new ddViewMenu(*this->Internal->ToolBarMenu, this);

  viewMenu->addWidget(
    this->Internal->ObjectsDock,
    this->Internal->ObjectsDock->windowTitle());

  viewMenu->addWidget(
    this->Internal->PropertiesDock,
    this->Internal->PropertiesDock->windowTitle());

  viewMenu->addWidget(
    this->Internal->OutputConsoleDock,
    this->Internal->OutputConsoleDock->windowTitle());

  toolbarMenu->addWidget(
    this->Internal->MainToolBar,
    this->Internal->MainToolBar->windowTitle());

}

//-----------------------------------------------------------------------------
void ddMainWindow::setPythonManager(ddPythonManager* pythonManager)
{
  this->Internal->PythonManager = pythonManager;
}

//-----------------------------------------------------------------------------
void ddMainWindow::setupPython()
{
  this->Internal->PythonManager->addObjectToPythonMain("_mainWindow", this);
  this->Internal->PythonManager->setupConsole(this);
  this->connect(this->Internal->ActionPythonConsole, SIGNAL(triggered()), this->Internal->PythonManager, SLOT(showConsole()));
}
