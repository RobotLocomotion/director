#ifndef __ddPythonEventFilter_h
#define __ddPythonEventFilter_h

#include <QObject>
#include <QEvent>
#include "ddAppConfigure.h"

class DD_APP_EXPORT ddPythonEventFilter : public QObject
{
  Q_OBJECT

public slots:

  void setEventHandlerResult(bool result)
  {
    this->EventHandlerResult = result;
  }

  void addFilteredEventType(int eventType)
  {
    this->EventTypes.append(eventType);
  }

  void removeFilteredEventType(int eventType)
  {
    this->EventTypes.removeAll(eventType);
  }

signals:

  void handleEvent(QObject* obj, QEvent* event);

protected:

  bool eventFilter(QObject *obj, QEvent *event)
  {
    if (this->EventTypes.contains(event->type()))
    {
        this->EventHandlerResult = false;
        emit this->handleEvent(obj, event);
        return this->EventHandlerResult;
    }
    return false;
  }

  bool EventHandlerResult;

  QList<int> EventTypes;
};

#endif
