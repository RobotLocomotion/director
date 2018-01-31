#include "ddROSPluginDecorators.h"

extern "C" void init_ddROSPlugin() {
  PythonQt::self()->addDecorators(new ddROSPluginDecorators);
}

