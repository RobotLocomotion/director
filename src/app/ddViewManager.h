#ifndef __ddViewManager_h
#define __ddViewManager_h

#include <QWidget>

class QTabWidget;
class ddViewBase;

class ddViewManager : public QWidget
{
    Q_OBJECT

public:

  ddViewManager(QWidget* parent=0);
  virtual ~ddViewManager();

  QTabWidget* tabWidget() const;

  ddViewBase* findView(const QString& viewName) const;


protected:

  void addDefaultPage();

  void addView(ddViewBase* view, const QString& viewName);

  class ddInternal;
  ddInternal* Internal;

  Q_DISABLE_COPY(ddViewManager);

};

#endif
