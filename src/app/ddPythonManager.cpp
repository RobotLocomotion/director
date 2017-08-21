#include "ddPythonManager.h"

#include "ddPythonQtWrapperFactory.h"
#include "ddPythonQtDecorators.h"

#include <ctkPythonConsole.h>

#include <QApplication>
#include <QDir>
#include <QShortcut>
#include <QLibrary>

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

  PythonQt::self()->importModule("PythonQt.dd").addObject("_pythonManager", this);

  QString libDir = QFileInfo(QCoreApplication::applicationDirPath() + "/../lib").canonicalFilePath();
  PythonQtObjectPtr mod = PythonQt::self()->importModule("sys");
  QVariantList version = mod.getVariable("version_info").value<QVariantList>();
  QString pythonMajor = version[0].toString();
  QString pythonMinor = version[1].toString();

  QStringList paths;
  paths << libDir + QString("/python%1/dist-packages").arg(pythonMajor);
  paths << libDir + QString("/python%1/site-packages").arg(pythonMajor);
  paths << libDir + QString("/python%1.%2/dist-packages").arg(pythonMajor, pythonMinor);
  paths << libDir + QString("/python%1.%2/site-packages").arg(pythonMajor, pythonMinor);

  foreach (const QString& path, paths)
  {
    if (QDir(path).exists())
    {
      PythonQt::self()->addSysPath(QDir::fromNativeSeparators(path));
    }
  }
}

//-----------------------------------------------------------------------------
QString ddPythonManager::appSitePackagesDir()
{
  PythonQtObjectPtr mod = PythonQt::self()->importModule("sys");
  QVariantList version = mod.getVariable("version_info").value<QVariantList>();

  QString sitePath = QString("%1/../lib/python%2.%3/site-packages").arg(
    QCoreApplication::applicationDirPath(),
    version[0].toString(),
    version[1].toString());

  return QFileInfo(sitePath).canonicalFilePath();
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
void ddPythonManager::loadPlugin(const QString& filename, const QString& functionName)
{
  QLibrary myLib(filename);
  if (!myLib.load())
  {
    printf("error loading library: %s\n", qPrintable(myLib.errorString()));
    return;
  }

  typedef void (*initFunction)();
  initFunction func = (initFunction) myLib.resolve(functionName.toLocal8Bit().data());
  if (func)
  {
    func();
  }
  else
  {
    printf("could not resolve init function: %s\n", qPrintable(functionName));
  }
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
