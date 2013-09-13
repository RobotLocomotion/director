#include <QApplication>

#include "ddMainWindow.h"


int main(int argc, char **argv)
{
  QApplication app(argc, argv);

  ddMainWindow window;
  window.show();

  return app.exec();
}
