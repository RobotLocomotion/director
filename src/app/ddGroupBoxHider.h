#ifndef __ddGroupBoxHider_h
#define __ddGroupBoxHider_h

#include <QObject>
#include <QGroupBox>
#include <QLayout>
#include <QLayoutItem>

class ddGroupBoxHider : public QObject
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

  static void setLayoutWidgetVisibility(QWidget* w, bool visible)
  {
    if (!w)
    {
      return;
    }

    QLayout* layout = w->layout();
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
    }
  }

public slots:

  void updateGroupBox()
  {
    QGroupBox* groupBox = this->parentGroupBox();
    this->setLayoutWidgetVisibility(groupBox, groupBox->isChecked());
  }

private:
  Q_DISABLE_COPY(ddGroupBoxHider);
};

#endif
