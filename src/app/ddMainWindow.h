#ifndef __ddMainWindow_h
#define __ddMainWindow_h

#include <QMainWindow>


class ddMainWindow : public QMainWindow
{
    Q_OBJECT

public:

  ddMainWindow();
  virtual ~ddMainWindow();

public slots:

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
