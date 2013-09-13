#include "ddMainWindow.h"

#include "ddDrakeModel.h"
#include "ddMacros.h"
#include "ddPythonManager.h"
#include "ddViewManager.h"
#include "ddPropertiesPanel.h"
#include "ddQVTKWidgetView.h"

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
  ddPropertiesPanel* PropertiesPanel;
  ddPythonManager* PythonManager;

  ddDrakeModel* DrakeModel;
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

  this->setupPython();

  QTimer::singleShot(0, this, SLOT(startup()));
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
void ddMainWindow::handleCommandLineArgs()
{
  QStringList args = QApplication::instance()->arguments();
  ddNotUsed(args);
}

//-----------------------------------------------------------------------------
void ddMainWindow::startup()
{
  this->handleCommandLineArgs();

  ddDrakeModel* model = new ddDrakeModel;
  this->Internal->DrakeModel = model;

  QString modelFile = "/home/pat/source/drc/drc-trunk/software/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf";
  model->loadFromFile(modelFile);

  ddQVTKWidgetView* view = qobject_cast<ddQVTKWidgetView*>(this->Internal->ViewManager->findView("VTK View"));
  model->addActorsToRenderer(view->renderer());
}

//-----------------------------------------------------------------------------
void ddMainWindow::setupPython()
{
  this->Internal->PythonManager = new ddPythonManager(this);
  this->Internal->PythonManager->addObjectToPythonMain("_mainWindow", this);
  this->Internal->PythonManager->setupConsole(this);
  this->connect(this->Internal->ActionPythonConsole, SIGNAL(triggered()), this->Internal->PythonManager, SLOT(showConsole()));
}
