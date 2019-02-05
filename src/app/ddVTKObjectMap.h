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
  // Puts the given vtkObjectBase instance into the map, and removes
  // any objects already in the map with this key.  The map takes ownership
  // of the given object.  This steals a reference. The caller should not
  // call Delete() or otherwise decrement the reference count after
  // the object is put into the map. The purpose of this function is
  // to pass the object to another thread, so you should not touch
  // the object after calling this function, or use anything like a
  // smart pointer that might touch the object to decrement a reference
  // count.  This is threadsafe and may be called from any thread.
  void put(const QString& key, vtkObjectBase* obj);

  // Description:
  // Same as put() except the object is appended to a list associated with
  // the given key.  See put() for ownership semantics and thread safety.
  void putAppend(const QString& key, vtkObjectBase* obj, bool sendSignal=true);

  // Description:
  // This function is designed to be called from Python.  This will
  // take the most recent vtkObjectBase instance in the map associated
  // with the given key (and erase other instances).  An empty QVariant
  // is returned if there are no instances in the map.  The returned instance
  // is wrapped with a reference counted Python object pointer using PythonQt's smart
  // pointer type PythonObjectPtr.  The PythonObjectPtr is returned
  // wrapped in a QVariant so that this header does not have any
  // dependency on the PythonQt API.  When Python releases the reference
  // the vtkObjectBase instance will be destroyed.  This function
  // should only be called from the main Python thread.
  // This is equivalent to takeAll().last() when the list is not empty.
  QVariant take(const QString& key) const;

  // Description:
  // This function is designed to be called from Python. See take() for
  // more details.  Returns a QList<QVariant>>, wrapped as a QVariant.
  // This is a list that contains all vtkObjectBase instances in the map
  // associated with the given key, in the order they were added to the
  // map.
  QVariant takeAll(const QString& key) const;

signals:

  // Description:
  // This signal is emitted each time put() or putAppend() is called, after the
  // reference has been stored in the map.  This signal is emitted
  // from the thread that called put() or putAppend().  You can connect to this
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
