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
  QMap<QString, vtkObjectBase*> Map;
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
QVariant ddVTKObjectMap::take(const QString& key) const
{
  QMutexLocker locker(&this->Internal->Mutex);
  PythonQtObjectPtr pythonPtr;
  vtkObjectBase* obj = this->Internal->Map.take(key);
  if (obj)
  {
    PythonQtObjectPtr pythonPtr;
    pythonPtr.setNewRef(vtkPythonUtil::GetObjectFromPointer(obj));
    obj->Delete();
    return QVariant::fromValue(pythonPtr);
  }
  return QVariant();
}

//-----------------------------------------------------------------------------
void ddVTKObjectMap::put(const QString& key, vtkObjectBase* obj)
{
  this->Internal->Mutex.lock();
  vtkObjectBase* prev = this->Internal->Map.value(key);
  if (prev)
  {
    prev->Delete();
  }
  this->Internal->Map[key] = obj;
  this->Internal->Mutex.unlock();
  emit this->objectAssigned(key);
}
