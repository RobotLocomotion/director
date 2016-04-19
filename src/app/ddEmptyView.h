#ifndef __ddEmptyView_h
#define __ddEmptyView_h

#include "ddViewBase.h"
#include "ddAppConfigure.h"



class DD_APP_EXPORT ddEmptyView : public ddViewBase
{
    Q_OBJECT

public:

  ddEmptyView(QWidget* parent=0);
  virtual ~ddEmptyView();

protected:

  class ddInternal;
  ddInternal* Internal;

  Q_DISABLE_COPY(ddEmptyView);
};

#endif
