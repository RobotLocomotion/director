#ifndef __ddPythonApp_h
#define __ddPythonApp_h

#include "ddAppConfigure.h"

#include <string>

class QApplication;

class DD_APP_EXPORT ddPythonApp
{
public:
  ddPythonApp();
  virtual ~ddPythonApp();

  bool init();
  bool init(QApplication* app);
  bool init(int &argc, char **argv);
  void start();
  void runPythonString(const std::string& str);

protected:

  class ddInternal;
  ddInternal* Internal;
};

#endif
