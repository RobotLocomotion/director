#ifndef __ddDrakeModel_h
#define __ddDrakeModel_h

#include <QObject>
#include <QColor>
#include <QVector>
#include "ddSharedPtr.h"
#include "ddAppConfigure.h"


class vtkRenderer;
class vtkTransform;
class vtkPolyData;
class RigidBodyManipulator;

class DD_APP_EXPORT ddDrakeModel : public QObject
{
    Q_OBJECT

public:

  ddDrakeModel(QObject* parent=0);
  virtual ~ddDrakeModel();

  bool loadFromFile(const QString& filename);
  bool loadFromXML(const QString& xmlString);
  const QString& filename() const;

  const ddSharedPtr<RigidBodyManipulator> getDrakeRBM() const;

  void addToRenderer(vtkRenderer* renderer);
  void removeFromRenderer(vtkRenderer* renderer);

  int numberOfJoints();
  void setJointPositions(const QVector<double>& positions, const QList<QString>& jointNames);
  void setJointPositions(const QVector<double>& positions);
  QVector<double> getJointPositions(const QList<QString>& jointNames) const;
  const QVector<double>& getJointPositions() const;
  QVector<double> getCenterOfMass() const;
  QVector<double> getJointLimits(const QString& jointName) const;
  QVector<double> getBodyContactPoints(const QString& bodyName) const;

  bool getLinkToWorld(const QString& linkName, vtkTransform* transform);
  QList<QString> getLinkNames();
  QList<QString> getJointNames();
  int findLinkID(const QString& linkName) const;

  void getModelMesh(vtkPolyData* polyData);

  QString getLinkNameForMesh(vtkPolyData* polyData);

  void setAlpha(double alpha);
  double alpha() const;

  void setTexturesEnabled(bool enabled);
  bool texturesEnabled() const;

  void setVisible(bool visible);
  bool visible() const;

  QColor color() const;
  void setColor(const QColor& color);

  void setLinkColor(const QString& linkName, const QColor& color);
  QColor getLinkColor(const QString& linkName) const;

  static void addPackageSearchPath(const QString& searchPath);
  static QString findPackageDirectory(const QString& packageName);

signals:

  void modelChanged();
  void displayChanged();

protected:

  class ddInternal;
  ddInternal* Internal;

  Q_DISABLE_COPY(ddDrakeModel);
};

#endif
