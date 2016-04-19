#ifndef __ddSignalMap_h
#define __ddSignalMap_h

#include <QObject>
#include <QMap>
#include <QVariant>
#include "ddAppConfigure.h"


class DD_APP_EXPORT ddSignalMap : public QObject
{
    Q_OBJECT

public:

  ddSignalMap(QObject* parent=0);
  virtual ~ddSignalMap();

  QMap<QString, QVariant> map() const;

  void signalItemChanged(const QString& key);
  void setItem(const QString& key, const QVariant& value);
  QVariant value(const QString& key) const;

signals:

  void itemAdded(const QString& key);
  void itemRemoved(const QString& key);
  void itemChanged(const QString& key);

protected:

  class ddInternal;
  ddInternal* Internal;

  Q_DISABLE_COPY(ddSignalMap);
};

#endif
