#ifndef __ddGLWidgetView_h
#define __ddGLWidgetView_h

#include "ddViewBase.h"


class ddGLWidgetView : public ddViewBase
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
