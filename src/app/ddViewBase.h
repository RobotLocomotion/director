#ifndef __ddViewBase_h
#define __ddViewBase_h

#include <QWidget>
#include "ddAppConfigure.h"


class DD_APP_EXPORT ddViewBase : public QWidget
{
    Q_OBJECT

public:

  ddViewBase(QWidget* parent=0);
  virtual ~ddViewBase();

protected:

  class ddInternal;
  ddInternal* Internal;

  Q_DISABLE_COPY(ddViewBase);
};

#endif
