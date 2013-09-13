#ifndef __ddMainWindow_h
#define __ddMainWindow_h

#include <QWidget>


class ddViewBase : public QWidget
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
