#ifndef __ddPythonManager_h
#define __ddPythonManager_h

#include <ctkAbstractPythonManager.h>

class ddPythonManager : public ctkAbstractPythonManager
{
    Q_OBJECT

public:

  ddPythonManager(QObject* parent=0);
  virtual ~ddPythonManager();

  void setupConsole(QWidget* parent);

  static QString appSitePackagesDir();

public slots:

  void showConsole();
  void onExecuteFile(const QString& filename);
  void handleCommandLineArgs();

protected:

  virtual void preInitialization();

  virtual QStringList pythonPaths();

  virtual void executeInitializationScripts();

  void setupConsoleShortcuts();

  class ddInternal;
  ddInternal* Internal;

  Q_DISABLE_COPY(ddPythonManager);
};

#endif
