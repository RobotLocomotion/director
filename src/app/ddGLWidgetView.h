#ifndef __ddGLWidgetView_h
#define __ddGLWidgetView_h

#include "ddViewBase.h"
#include "ddAppConfigure.h"


class DD_APP_EXPORT ddGLWidgetView : public ddViewBase
{
    Q_OBJECT

public:

  ddGLWidgetView(QWidget* parent=0);
  virtual ~ddGLWidgetView();

protected:

  class ddInternal;
  ddInternal* Internal;

  Q_DISABLE_COPY(ddGLWidgetView);
};

#endif
