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

  bool loadFromFile(const QString& filename);
  const QString& filename() const;

  void addToRenderer(vtkRenderer* renderer);
  void removeFromRenderer(vtkRenderer* renderer);

  int numberOfJoints();
  void setJointPositions(const QList<double>& positions);

  void setEstRobotState(const QList<double>& robotState);

  void setAlpha(double alpha);
  double alpha() const;

  void setVisible(bool visible);
  bool visible() const;

signals:

  void modelChanged();

protected:

  class ddInternal;
  ddInternal* Internal;

  Q_DISABLE_COPY(ddDrakeModel);
};

#endif
