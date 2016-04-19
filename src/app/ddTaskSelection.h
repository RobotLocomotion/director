#ifndef __ddTaskSelection_h
#define __ddTaskSelection_h

#include <QWidget>
#include "ddAppConfigure.h"


class DD_APP_EXPORT ddTaskSelection : public QWidget
{
    Q_OBJECT

public:

  ddTaskSelection();
  virtual ~ddTaskSelection();



signals:

  void taskSelected(int taskId);


protected slots:

  void onButtonClicked();

protected:

  class ddInternal;
  ddInternal* Internal;

  Q_DISABLE_COPY(ddTaskSelection);
};

#endif
