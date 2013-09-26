#include "ddPythonManager.h"

#include "ddPythonQtWrapperFactory.h"
#include "ddPythonQtDecorators.h"

#include <ctkPythonConsole.h>

#include <QApplication>
#include <QDir>
#include <QShortcut>

//-----------------------------------------------------------------------------
class ddPythonManager::ddInternal
{
public:

  QPointer<ctkPythonConsole> Console;
};


//-----------------------------------------------------------------------------
ddPythonManager::ddPythonManager(QObject* parent) : ctkAbstractPythonManager(parent)
{
  this->Internal = new ddInternal;

  ctkPythonConsole* console = new ctkPythonConsole;
  console->setWindowFlags(Qt::Dialog | Qt::WindowStaysOnTopHint);
  console->initialize(this);
  console->setAttribute(Qt::WA_QuitOnClose, true);
  console->resize(800, 400);
  console->setProperty("isInteractive", true);
  this->Internal->Console = console;
  this->setupConsoleShortcuts();
  this->addObjectToPythonMain("_console", console);
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
QString ddPythonManager::appSitePackagesDir() const
{
  return QCoreApplication::applicationDirPath()  + "/../lib/site-packages";
}

//-----------------------------------------------------------------------------
QStringList ddPythonManager::pythonPaths()
{
  QString rootDir = "/source";
  if (!QDir(rootDir).exists())
  {
    rootDir = QDir::homePath() + rootDir;
  }

  QStringList searchDirs;
  searchDirs << this->appSitePackagesDir()
             << rootDir + "/paraview/build/VTK/Wrapping/Python"
             << rootDir + "/paraview/build/lib"
             << rootDir + "/paraview/PCLPlugin/build/lib";

  QStringList paths;
  foreach (const QString& dirname, searchDirs)
  {
    if (QDir(dirname).exists())
    {
      paths.append(dirname);
    }
  }
  return paths;
}

//-----------------------------------------------------------------------------
void ddPythonManager::executeInitializationScripts()
{

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
