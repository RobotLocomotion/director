#ifndef __ddPythonQtWrapperFactory_h
#define __ddPythonQtWrapperFactory_h

#include <PythonQtCppWrapperFactory.h>
#include "ddAppConfigure.h"


class DD_APP_EXPORT ddPythonQtWrapperFactory : public PythonQtForeignWrapperFactory
{
public:
  virtual PyObject* wrap(const QByteArray& classname, void *ptr);
  virtual void* unwrap(const QByteArray& classname, PyObject* object);
};

#endif
