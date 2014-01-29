#include "ddSignalMap.h"


#include <QMap>


//-----------------------------------------------------------------------------
class ddSignalMap::ddInternal
{
public:

  ddInternal()
  {

  }

  QMap<QString, QVariant> Map;

};

//-----------------------------------------------------------------------------
ddSignalMap::ddSignalMap(QObject* parent) : QObject(parent)
{
  this->Internal = new ddInternal;
}

//-----------------------------------------------------------------------------
ddSignalMap::~ddSignalMap()
{
  delete this->Internal;
}

//-----------------------------------------------------------------------------
QMap<QString, QVariant> ddSignalMap::map() const
{
  return this->Internal->Map;
}

//-----------------------------------------------------------------------------
void ddSignalMap::signalItemChanged(const QString& key)
{
  emit this->itemChanged(key);
}

//-----------------------------------------------------------------------------
void ddSignalMap::setItem(const QString& key, const QVariant& value)
{
  if (this->Internal->Map.contains(key))
  {
    this->Internal->Map[key] = value;
    this->signalItemChanged(key);
  }
  else
  {
    this->Internal->Map[key] = value;
    emit this->itemAdded(key);
  }
}

//-----------------------------------------------------------------------------
QVariant ddSignalMap::value(const QString& key) const
{
  return this->Internal->Map.value(key);
}
