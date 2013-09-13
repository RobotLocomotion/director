#ifndef __ddDrakeModel_h
#define __ddDrakeModel_h

#include <QObject>

class vtkRenderer;

class ddDrakeModel : public QObject
{
    Q_OBJECT

public:

  ddDrakeModel(QObject* parent=0);
  virtual ~ddDrakeModel();

  void loadFromFile(const QString& filename);
  void addActorsToRenderer(vtkRenderer* renderer);

protected:

  class ddInternal;
  ddInternal* Internal;

  Q_DISABLE_COPY(ddDrakeModel);
};

#endif
