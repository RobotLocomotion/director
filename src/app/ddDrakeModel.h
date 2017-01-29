#ifndef __ddDrakeModel_h
#define __ddDrakeModel_h

#include <QObject>
#include <QColor>
#include <QVector>
#include <QGenericMatrix>
#include "ddSharedPtr.h"
#include "ddAppConfigure.h"
#include "ddDrakeVersion.h"
#ifdef DRAKE_OH_FORK
#include <drake/systems/plants/RigidBodyTree.h>
#else
#include <drake/multibody/rigid_body_tree.h>
#endif


class vtkRenderer;
class vtkTransform;
class vtkPolyData;

#ifdef DRAKE_OH_FORK
#define RigidBodyTreed RigidBodyTree
#endif

class DD_APP_EXPORT ddDrakeModel : public QObject
{
    Q_OBJECT

public:

  ddDrakeModel(QObject* parent=0);
  virtual ~ddDrakeModel();

  bool loadFromFile(const QString& filename);
  bool loadFromXML(const QString& xmlString);
  const QString& filename() const;

  const ddSharedPtr<RigidBodyTreed> getDrakeRBM() const;
  const ddSharedPtr<KinematicsCache<double> > getKinematicsCache() const;

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
  void doKinematics(const QVector<double>& q, const QVector<double>& v, bool compute_gradients = false,
  bool compute_JdotV = true);

  QVector<double> geometricJacobian(int base_body_or_frame_ind, int end_effector_body_or_frame_ind, int expressed_in_body_or_frame_ind, int gradient_order, bool in_terms_of_qdot = false);


  bool getLinkToWorld(const QString& linkName, vtkTransform* transform);
  bool getFrameToWorld(int frameId, vtkTransform* transform);
  QList<QString> getLinkNames();
  QList<QString> getJointNames();
  int findLinkID(const QString& linkName) const;
  int findJointID(const QString& jointName) const;
  int findFrameID(const QString& frameName) const;

  void getModelMesh(vtkPolyData* polyData);
  void getModelMeshWithLinkInfoAndNormals(vtkPolyData* polyData);
  void getLinkModelMesh(const QString& linkName, vtkPolyData* polyData);
  QString getLinkNameForMesh(vtkPolyData* polyData);
  QString getBodyOrFrameName(int body_or_frame_id);

  void setAlpha(double alpha);
  double alpha() const;

  void setUrdfColors();

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
