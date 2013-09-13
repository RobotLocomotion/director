#ifndef __ddPropertiesPanel_h
#define __ddPropertiesPanel_h

#include <QWidget>


class ddPropertiesPanel : public QWidget
{
    Q_OBJECT

public:

  ddPropertiesPanel(QWidget* parent=0);
  virtual ~ddPropertiesPanel();

protected:

  class ddInternal;
  ddInternal* Internal;

  Q_DISABLE_COPY(ddPropertiesPanel);
};

#endif
