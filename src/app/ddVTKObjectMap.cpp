#include "ddVTKObjectMap.h"

#include <PythonQtObjectPtr.h>
#include <QMap>
#include <QMutex>
#include <QMutexLocker>
#include <vtkPythonUtil.h>
#include <vtkObjectBase.h>


//-----------------------------------------------------------------------------
class ddVTKObjectMap::ddInternal
{
public:
  QMap<QString, QList<vtkObjectBase*>> Map;
  QMutex Mutex;
};

//-----------------------------------------------------------------------------
ddVTKObjectMap::ddVTKObjectMap(QObject* parent) : QObject(parent)
{
  this->Internal = new ddInternal;
}

//-----------------------------------------------------------------------------
ddVTKObjectMap::~ddVTKObjectMap()
{
  delete this->Internal;
}

//-----------------------------------------------------------------------------
QVariant ddVTKObjectMap::takeAll(const QString& key) const
{
  QMutexLocker locker(&this->Internal->Mutex);
  QList<QVariant> pyList;
  QList<vtkObjectBase*> objList = this->Internal->Map.take(key);
  foreach(vtkObjectBase* obj, objList)
  {
    PythonQtObjectPtr pythonPtr;
    pythonPtr.setNewRef(vtkPythonUtil::GetObjectFromPointer(obj));
    pyList.append(QVariant::fromValue(pythonPtr));
    obj->Delete();
  }
  return pyList;
}

//-----------------------------------------------------------------------------
QVariant ddVTKObjectMap::take(const QString& key) const
{
  QList<QVariant> pyList = this->takeAll(key).value<QList<QVariant>>();
  return pyList.count() ?  QVariant::fromValue(pyList.last()) : QVariant();
}

//-----------------------------------------------------------------------------
void ddVTKObjectMap::put(const QString& key, vtkObjectBase* obj)
{
  this->Internal->Mutex.lock();
  foreach(vtkObjectBase* obj, this->Internal->Map.take(key))
  {
    obj->Delete();
  }
  QList<vtkObjectBase*> newList;
  newList.append(obj);
  this->Internal->Map[key] = newList;
  this->Internal->Mutex.unlock();
  emit this->objectAssigned(key);
}


//-----------------------------------------------------------------------------
void ddVTKObjectMap::putAppend(const QString& key, vtkObjectBase* obj, bool sendSignal)
{
  this->Internal->Mutex.lock();
  if (this->Internal->Map.contains(key))
  {
    this->Internal->Map[key].append(obj);
  }
  else
  {
    QList<vtkObjectBase*> newList;
    newList.append(obj);
    this->Internal->Map[key] = newList;
  }

  this->Internal->Mutex.unlock();

  if (sendSignal)
  {
    emit this->objectAssigned(key);
  }
}
