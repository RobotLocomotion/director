#include "ddPythonQtWrapperFactory.h"

#include <vtkPythonUtil.h>
#include <vtkObject.h>

//-----------------------------------------------------------------------------
PyObject* ddPythonQtWrapperFactory::wrap(const QByteArray& classname, void *ptr)
{
  if (classname.startsWith("vtk"))
    {
    return vtkPythonUtil::GetObjectFromPointer(static_cast<vtkObjectBase*>(ptr));
    }
  return NULL;
}

//-----------------------------------------------------------------------------
void* ddPythonQtWrapperFactory::unwrap(const QByteArray& classname, PyObject* object)
{
  if (classname.startsWith("vtk"))
    {
    return vtkPythonUtil::GetPointerFromObject(object, classname.data());
    }
  return NULL;
}
