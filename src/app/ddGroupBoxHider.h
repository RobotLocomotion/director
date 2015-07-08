#ifndef __ddGroupBoxHider_h
#define __ddGroupBoxHider_h

#include <QObject>
#include <QGroupBox>
#include <QLayout>
#include <QLayoutItem>
#include "ddAppConfigure.h"


class DD_APP_EXPORT ddGroupBoxHider : public QObject
{
    Q_OBJECT

public:

  ddGroupBoxHider(QGroupBox* parent) : QObject(parent)
  {
    this->setParent(parent);
    this->connect(parent, SIGNAL(toggled(bool)), SLOT(updateGroupBox()));
    if (!parent->isCheckable())
    {
      parent->setCheckable(true);
    }
    this->updateGroupBox();
  }

  ~ddGroupBoxHider()
  {

  }

  QGroupBox* parentGroupBox()
  {
    return qobject_cast<QGroupBox*>(this->parent());
  }

  static void setLayoutWidgetVisibility(QLayout* layout, bool visible)
  {
    if (!layout)
    {
      return;
    }

    for (int i = 0; i < layout->count(); ++i)
    {
      QLayoutItem* item = layout->itemAt(i);
      QWidget* layoutWidget = item->widget();
      if (layoutWidget)
      {
        layoutWidget->setVisible(visible);
      }
      else
      {
        setLayoutWidgetVisibility(item->layout(), visible);
      }
    }
  }

public slots:

  void updateGroupBox()
  {
    QGroupBox* groupBox = this->parentGroupBox();
    this->setLayoutWidgetVisibility(groupBox->layout(), groupBox->isChecked());
  }

private:
  Q_DISABLE_COPY(ddGroupBoxHider);
};

#endif
