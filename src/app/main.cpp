#include <QApplication>
#include <QVTKApplication.h>

#include "ddMainWindow.h"


int main(int argc, char **argv)
{
  QVTKApplication app(argc, argv);

  ddMainWindow window;
  window.resize(1800, 1000);
  window.show();

  return app.exec();
}
