#include <PythonQt.h>

#include <QMainWindow>
#include <QPushButton>
#include <QVariant>
#include <QDebug>


#include <QRunnable>
#include <chrono>
#include <thread>



class MyRunObject : public QRunnable
{
  virtual void run()
  {
    printf("running...\n");
    for (int i = 0; i < 10; ++i) {
      printf("iteration %d\n", i);
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));

      PythonQtObjectPtr main = PythonQt::self()->getMainModule();
      main.evalScript("x = x + 1.0");
    }
  }

  virtual ~MyRunObject()
  {
    printf("MyRunObject destructor\n");
  }

};
Q_DECLARE_METATYPE(QRunnable*);


void init()
{
  QPushButton* button = new QPushButton("My Button");
  PythonQtObjectPtr main = PythonQt::self()->getMainModule();
  main.addVariable("my_button", QVariant::fromValue((QObject*)button));
  main.addVariable("x", QVariant::fromValue(1.0));

  PythonQtObjectPtr myModule = PythonQt::self()->createModuleFromScript("my_module");
  myModule.addVariable("my_run", QVariant::fromValue((QRunnable*)new MyRunObject));

  myModule.evalScript(
    "from PythonQt import QtCore\n"
    "def run():\n"
    "   QtCore.QThreadPool.globalInstance().start(my_run)\n");


  QVariant val = main.getVariable("mainWindow");
  QMainWindow* mainWindow = qobject_cast<QMainWindow*>(val.value<QObject*>());
  if (mainWindow)
  {
    mainWindow->setWindowTitle("My Main Window");
  }


}


// ------------------------------------
extern "C" void init_my_library() {
  init();
}
