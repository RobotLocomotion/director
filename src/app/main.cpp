#include <QApplication>

#include "ddMainWindow.h"


int main(int argc, char **argv)
{
  QApplication app(argc, argv);

  ddMainWindow window;
  window.resize(1800, 1000);
  window.show();

  return app.exec();
}
