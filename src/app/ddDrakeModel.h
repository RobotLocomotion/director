#ifndef __ddDrakeModel_h
#define __ddDrakeModel_h

#include <QObject>
#include <QColor>

class vtkRenderer;
class vtkTransform;
class vtkPolyData;

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
  void setJointPositions(const QList<double>& positions, const QList<QString>& jointNames);
  void setJointPositions(const QList<double>& positions);

  void setEstRobotState(const QList<double>& robotState);

  bool getLinkToWorld(const QString& linkName, vtkTransform* transform);
  QList<QString> getLinkNames();
  QList<QString> getJointNames();

  void getModelMesh(vtkPolyData* polyData);

  QString getLinkNameForMesh(vtkPolyData* polyData);

  void setAlpha(double alpha);
  double alpha() const;

  void setVisible(bool visible);
  bool visible() const;

  QColor color() const;
  void setColor(const QColor& color);

  void setLinkColor(const QString& linkName, const QColor& color);

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
