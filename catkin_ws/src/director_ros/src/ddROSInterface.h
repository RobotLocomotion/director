#ifndef __ddROSInterface_h
#define __ddROSInterface_h

#include <QObject>

class ddVTKObjectMap;

class ddROSInterface : public QObject {
  Q_OBJECT

 public:
  ddROSInterface();
  virtual ~ddROSInterface();

  void addPointcloudSubscriber(const QString& topicName);
  void addImageSubscriber(const QString& topicName);

  void initNode();
  bool rosCheckMaster();
  void rosRequestShutdown();

  void startSpinner();
  void stopSpinner();

  ddVTKObjectMap* vtkObjectMap();


 protected:
  class Internal;
  Internal* internal_;

  Q_DISABLE_COPY(ddROSInterface);
};

#endif  // __ddROSInterface_h
