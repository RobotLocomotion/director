#ifndef __ddLumberSelection_h
#define __ddLumberSelection_h

#include <QWidget>
#include "ddAppConfigure.h"


class DD_APP_EXPORT ddLumberSelection : public QWidget
{
    Q_OBJECT

public:

  ddLumberSelection();
  virtual ~ddLumberSelection();


  enum LumberId
  {
    TwoByFour = 0,
    TwoBySix = 1,
    FourByFour = 2
  };

signals:

  void lumberSelected(int lumberId);

protected slots:

  void onButtonClicked();

protected:

  class ddInternal;
  ddInternal* Internal;

  Q_DISABLE_COPY(ddLumberSelection);
};

#endif
