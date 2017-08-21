#include <ddPythonApp.h>
#include <iostream>


int main(int argc, char **argv)
{
  ddPythonApp app;
  bool result = app.init(argc, argv);
  if (!result) {
    std::cout << "app.init() failed." << std::endl;
    return 1;
  }

  app.runPythonString("print('hello world!')");
  app.start();
  return 0;
}
