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
  this->addObjectToPythonMain("_pythonManager", this);
  this->addObjectToPythonMain("_console", console);
}

//-----------------------------------------------------------------------------
ddPythonManager::~ddPythonManager()
{
  delete this->Internal->Console;
  delete this->Internal;
}

//-----------------------------------------------------------------------------
void ddPythonManager::handleCommandLineArgs()
{
  QStringList args = QApplication::instance()->arguments();
  args.removeFirst();
  foreach (QString arg, args)
  {
    this->executeFile(arg);
  }
}

//-----------------------------------------------------------------------------
void ddPythonManager::preInitialization()
{
  this->addWrapperFactory(new ddPythonQtWrapperFactory);
  this->registerPythonQtDecorator(new ddPythonQtDecorators);

  // when running from a cmake build directory (not an install tree)
  // then automatically prepend the python sys.path
  if (QFileInfo(QCoreApplication::applicationDirPath()  + "/../CMakeCache.txt").exists())
  {
    PythonQt::self()->addSysPath(this->appSitePackagesDir());
  }

  PythonQtObjectPtr mod = PythonQt::self()->importModule("PythonQt.dd");
  mod.addObject("_pythonManager", this);
}

//-----------------------------------------------------------------------------
QString ddPythonManager::appSitePackagesDir()
{
  return QFileInfo(QCoreApplication::applicationDirPath()  + "/../lib/python2.7/dist-packages").canonicalFilePath();
}

//-----------------------------------------------------------------------------
QStringList ddPythonManager::pythonPaths()
{
  QStringList searchDirs;
  searchDirs << this->appSitePackagesDir();
  searchDirs << this->appSitePackagesDir().replace("dist-packages", "site-packages");

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
void ddPythonManager::setSysArgv(const QStringList& args)
{
  PythonQtObjectPtr mod = PythonQt::self()->importModule("sys");
  if (mod)
    {
    return PythonQt::self()->addVariable(mod, "argv", args);
    }
}

//-----------------------------------------------------------------------------
void ddPythonManager::executeInitializationScripts()
{

}

//-----------------------------------------------------------------------------
ctkPythonConsole* ddPythonManager::consoleWidget() const
{
  return this->Internal->Console;
}

//-----------------------------------------------------------------------------
void ddPythonManager::showConsole()
{
  this->Internal->Console->show();
}

//-----------------------------------------------------------------------------
void ddPythonManager::onExecuteFile(const QString& filename)
{
  this->executeFile(filename);
}

//-----------------------------------------------------------------------------
void ddPythonManager::setupConsole(QWidget* parent)
{
  this->Internal->Console->setParent(parent);
#ifdef Q_OS_MAC
    this->Internal->Console->setWindowFlags(Qt::Dialog | Qt::WindowStaysOnTopHint);
#else
    this->Internal->Console->setWindowFlags(Qt::Dialog);
#endif
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
