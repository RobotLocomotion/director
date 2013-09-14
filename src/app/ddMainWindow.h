#ifndef __ddMainWindow_h
#define __ddMainWindow_h

#include <QMainWindow>

class ddViewManager;
class ddPropertiesPanel;

class ddMainWindow : public QMainWindow
{
    Q_OBJECT

public:

  ddMainWindow();
  virtual ~ddMainWindow();

  ddViewManager* viewManager() const;
  ddPropertiesPanel* propertiesPanel() const;
  QToolBar* toolBar() const;

protected slots:

  void startup();

protected:

  void handleCommandLineArgs();
  void setupPython();

  class ddInternal;
  ddInternal* Internal;

  Q_DISABLE_COPY(ddMainWindow);
};

#endif
