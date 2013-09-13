#ifndef __ddViewManager_h
#define __ddViewManager_h

#include <QWidget>

class QTabWidget;

class ddViewManager : public QWidget
{
    Q_OBJECT

public:

  ddViewManager(QWidget* parent=0);
  virtual ~ddViewManager();

  QTabWidget* tabWidget() const;

public slots:

protected:

  void addDefaultPage();

  class ddInternal;
  ddInternal* Internal;

  Q_DISABLE_COPY(ddViewManager);

};

#endif
