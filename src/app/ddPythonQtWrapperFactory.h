#ifndef __ddPythonQtWrapperFactory_h
#define __ddPythonQtWrapperFactory_h

#include <PythonQtCppWrapperFactory.h>

class ddPythonQtWrapperFactory : public PythonQtForeignWrapperFactory
{
public:
  virtual PyObject* wrap(const QByteArray& classname, void *ptr);
  virtual void* unwrap(const QByteArray& classname, PyObject* object);
};

#endif
