#ifndef __ddVTKObjectMap_h
#define __ddVTKObjectMap_h

#include <QObject>
#include <QMap>
#include <QVariant>
#include "ddAppConfigure.h"

class vtkObjectBase;

class DD_APP_EXPORT ddVTKObjectMap : public QObject
{
    Q_OBJECT

public:

  ddVTKObjectMap(QObject* parent=0);
  virtual ~ddVTKObjectMap();

  // Description:
  // Puts the given vtkObjectBase instance into the map and takes
  // ownership.  This steals a reference. The caller should not
  // call Delete() or otherwise decrement the reference count after
  // the object is put into the map. The purpose of this function is
  // to pass the object to another thread, so you should should touch
  // the object after calling this function, or use anything like a
  // smart pointer that might touch the object to decrement a reference
  // count.  This is threadsafe and may be called from any thread.
  void put(const QString& key, vtkObjectBase* obj);

  // Description:
  // This function is designed to be called from Python.  This will
  // remove the vtkObjectBase instance from the map and wrap it with
  // a reference counted Python object pointer using PythonQt's smart
  // pointer type PythonObjectPtr.  The PythonObjectPtr is returned
  // wrapped in a QVariant so that this header does not have any
  // dependency on the PythonQt API.  When Python releases the reference
  // the vtkObjectBase instance will be destroyed.  This function
  // should only be called from the main Python thread.
  QVariant take(const QString& key) const;

signals:

  // Description:
  // This signal is emitted each time put() is called, after the
  // reference has been stored in the map.  This signal is emitted
  // from the thread that called put().  You can connect to this
  // signal in Python on the main thread to get a notification.
  // Note that the signal connection will be queued so you may get
  // multiple signals emitted before the signal handler is called.
  void objectAssigned(const QString& key);

protected:

  class ddInternal;
  ddInternal* Internal;

  Q_DISABLE_COPY(ddVTKObjectMap);
};

#endif
