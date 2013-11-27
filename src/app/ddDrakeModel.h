#ifndef __ddDrakeModel_h
#define __ddDrakeModel_h

#include <QObject>

class vtkRenderer;
class vtkTransform;

class ddDrakeModel : public QObject
{
    Q_OBJECT

public:

  ddDrakeModel(QObject* parent=0);
  virtual ~ddDrakeModel();

  bool loadFromFile(const QString& filename);
  bool loadFromXML(const QString& xmlString);
  const QString& filename() const;

  void addToRenderer(vtkRenderer* renderer);
  void removeFromRenderer(vtkRenderer* renderer);

  int numberOfJoints();
  void setJointPositions(const QList<double>& positions);

  void setEstRobotState(const QList<double>& robotState);

  void getLinkToWorld(const QString& linkName, vtkTransform* transform);
  QList<QString> getLinkNames();

  void setAlpha(double alpha);
  double alpha() const;

  void setVisible(bool visible);
  bool visible() const;

  static void addPackageSearchPath(const QString& searchPath);
  static QString findPackageDirectory(const QString& packageName);

signals:

  void modelChanged();

protected:

  class ddInternal;
  ddInternal* Internal;

  Q_DISABLE_COPY(ddDrakeModel);
};

#endif
