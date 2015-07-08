#ifndef __ddMainWindow_h
#define __ddMainWindow_h

#include <QMainWindow>
#include "ddAppConfigure.h"


class ddViewManager;
class ddPropertiesPanel;
class ddPythonManager;
class ddViewBase;
class ddObjectTree;
class QAction;
class QTextEdit;
class QDockWidget;
class QMenu;

class DD_APP_EXPORT ddMainWindow : public QMainWindow
{
    Q_OBJECT

public:

  ddMainWindow();
  virtual ~ddMainWindow();

  ddViewManager* viewManager() const;
  ddPropertiesPanel* propertiesPanel() const;
  ddObjectTree* objectTree() const;
  QToolBar* toolBar() const;
  QToolBar* macrosToolBar() const;
  QTextEdit* outputConsole() const;

  void addWidgetToViewMenu(QWidget* widget);
  void addWidgetToViewMenu(QWidget* widget, QAction* action);

  QList<QAction*> toolBarActions() const;

  void setPythonManager(ddPythonManager* pythonManager);

  QMenu* toolsMenu() const;

signals:

  void resetCamera();
  void toggleStereoRender();
  void toggleCameraTerrainMode();
  void fileOpen();
  void fileSaveData();
  void fileExportUrdf();


protected slots:

  void startup();
  void toggleOutputConsoleVisibility();

  void onCurrentViewChanged(ddViewBase* previousView, ddViewBase* currentView);

protected:

  void handleCommandLineArgs();
  void setupPython();
  void setupViewMenu();
  virtual void closeEvent(QCloseEvent *event);

  class ddInternal;
  ddInternal* Internal;

  Q_DISABLE_COPY(ddMainWindow);
};

#endif
