#include "ddDrakeWrapper.h"
#include "ddSharedPtr.h"

#include <RigidBodyManipulator.h>
#include <convexHull.h>
#include <ForceTorqueMeasurement.h>
#include <map>
#include <vector>
#include <unordered_set>
#include <string>
#include <sstream>
#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/find.hpp>
#include <boost/algorithm/string/case_conv.hpp>
#include <boost/algorithm/string/replace.hpp>

#include <math.h>

#include <QMap>
#include <QDir>

using std::map;
using std::vector;
using std::string;
using std::istringstream;

//-----------------------------------------------------------------------------
class ddDrakeWrapper::ddInternal
{
public:

};


//-----------------------------------------------------------------------------
ddDrakeWrapper::ddDrakeWrapper(QObject* parent) : QObject(parent)
{
  this->Internal = new ddInternal;
}

//-----------------------------------------------------------------------------
ddDrakeWrapper::~ddDrakeWrapper()
{
  delete this->Internal;
}

//-----------------------------------------------------------------------------
QVector<double> ddDrakeWrapper::resolveCenterOfPressure(const ddDrakeModel& ddModel, 
    const QVector<int>& ft_frame_ids, 
    const QVector<double> & ft_in, 
    const QVector<double> & normal_in, 
    const QVector<double> & point_on_contact_plane_in) const
{
  // Assumes size of ft_in = size of ft_frame_inds*6, in order (one wrench at a time)
  // returns a 4-vector, which is packed output of RBM resolveCOP, with vector first
  ddSharedPtr<RigidBodyManipulator> model = ddModel.getDrakeRBM();
  Vector3d normal; for (int i=0; i<normal_in.size(); i++) normal[i] = normal_in[i];
  Vector3d point_on_contact_plane; for (int i=0; i<point_on_contact_plane_in.size(); i++) point_on_contact_plane[i] = point_on_contact_plane_in[i];
  std::vector<ForceTorqueMeasurement> force_torque_measurements;
  for (int i=0; i<ft_frame_ids.size(); i++){
    ForceTorqueMeasurement ft;
    ft.frame_idx = ft_frame_ids[i];
    ft.wrench = Eigen::Matrix<double, 6, 1>();
    for (int j=0; j<6; j++) ft.wrench[j] = ft_in[i*6+j];
    force_torque_measurements.push_back(ft);
  }
  std::pair<Eigen::Vector3d, double> ret_eigen = model->resolveCenterOfPressure(force_torque_measurements, normal, point_on_contact_plane);
  QVector<double> ret; for (int i=0; i<3; i++) ret << ret_eigen.first[i];
  ret << ret_eigen.second;
  return ret;
}

//-----------------------------------------------------------------------------
double ddDrakeWrapper::drakeSignedDistanceInsideConvexHull(int num_pts, const QVector<double>& pts_in, const QVector<double> & q_in) const
{
  Matrix<double, 2, Eigen::Dynamic> pts(2, num_pts);
  for (int i=0; i<num_pts; i++){
    pts(0,i) = pts_in[i*2];
    pts(1,i) = pts_in[i*2+1];
  }
  Vector2d q; q[0] = q_in[0]; q[1] = q_in[1];
  return signedDistanceInsideConvexHull(pts, q);
}