#include "ddDrakeModel.h"
#include "ddSharedPtr.h"

#include <URDFRigidBodyManipulator.h>

#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataNormals.h>
#include <vtkRenderer.h>
#include <vtkOBJReader.h>
#include <vtkSphereSource.h>
#include <vtkCylinderSource.h>
#include <vtkCubeSource.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkTransform.h>
#include <vtkMath.h>
//#include <vtkQuaternion.h>
#include <vtkProperty.h>

#include <map>
#include <vector>
#include <string>
#include <sstream>
#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/replace.hpp>

#include <urdf_interface/model.h>

#include <math.h>

#include <QMap>
#include <QDir>

using std::map;
using std::vector;
using std::string;
using std::istringstream;

#if VTK_MAJOR_VERSION == 6
 #define SetInputData(filter, obj) filter->SetInputData(obj);
#else
  #define SetInputData(filter, obj) filter->SetInput(obj);
#endif

class ddMeshVisual
{
 public:

  ddPtrMacro(ddMeshVisual);
  ddMeshVisual()
  {

  }

  vtkSmartPointer<vtkPolyData> PolyData;
  vtkSmartPointer<vtkActor> Actor;
  vtkSmartPointer<vtkTransform> Transform;
  std::string Name;

private:

  Q_DISABLE_COPY(ddMeshVisual);
};

namespace
{


std::vector<std::string> getEstRobotStateJointNames()
{
  std::vector<std::string> names;
  names.push_back("back_bkz");
  names.push_back("back_bky");
  names.push_back("back_bkx");
  names.push_back("neck_ay");
  names.push_back("l_leg_hpz");
  names.push_back("l_leg_hpx");
  names.push_back("l_leg_hpy");
  names.push_back("l_leg_kny");
  names.push_back("l_leg_aky");
  names.push_back("l_leg_akx");
  names.push_back("r_leg_hpz");
  names.push_back("r_leg_hpx");
  names.push_back("r_leg_hpy");
  names.push_back("r_leg_kny");
  names.push_back("r_leg_aky");
  names.push_back("r_leg_akx");
  names.push_back("l_arm_usy");
  names.push_back("l_arm_shx");
  names.push_back("l_arm_ely");
  names.push_back("l_arm_elx");
  names.push_back("l_arm_uwy");
  names.push_back("l_arm_mwx");
  names.push_back("r_arm_usy");
  names.push_back("r_arm_shx");
  names.push_back("r_arm_ely");
  names.push_back("r_arm_elx");
  names.push_back("r_arm_uwy");
  names.push_back("r_arm_mwx");
  names.push_back("hokuyo_joint");

  /*
  names.push_back("pre_spindle_cal_x_joint");
  names.push_back("pre_spindle_cal_y_joint");
  names.push_back("pre_spindle_cal_z_joint");
  names.push_back("pre_spindle_cal_roll_joint");
  names.push_back("pre_spindle_cal_pitch_joint");
  names.push_back("pre_spindle_cal_yaw_joint");
  names.push_back("post_spindle_cal_x_joint");
  names.push_back("post_spindle_cal_y_joint");
  names.push_back("post_spindle_cal_z_joint");
  names.push_back("post_spindle_cal_roll_joint");
  names.push_back("post_spindle_cal_pitch_joint");
  names.push_back("post_spindle_cal_yaw_joint");
  */

  return names;
}

std::vector<std::string> getDofNames()
{
  std::vector<std::string> names;
  names.push_back("base_x");
  names.push_back("base_y");
  names.push_back("base_z");
  names.push_back("base_roll");
  names.push_back("base_pitch");
  names.push_back("base_yaw");
  names.push_back("back_bkz");
  names.push_back("back_bky");
  names.push_back("back_bkx");
  names.push_back("l_arm_usy");
  names.push_back("l_arm_shx");
  names.push_back("l_arm_ely");
  names.push_back("l_arm_elx");
  names.push_back("l_arm_uwy");
  names.push_back("l_leg_hpz");
  names.push_back("l_leg_hpx");
  names.push_back("l_leg_hpy");
  names.push_back("l_leg_kny");
  names.push_back("l_leg_aky");
  names.push_back("l_leg_akx");
  names.push_back("l_arm_mwx");
  names.push_back("r_arm_usy");
  names.push_back("r_arm_shx");
  names.push_back("r_arm_ely");
  names.push_back("r_arm_elx");
  names.push_back("r_arm_uwy");
  names.push_back("r_leg_hpz");
  names.push_back("r_leg_hpx");
  names.push_back("r_leg_hpy");
  names.push_back("r_leg_kny");
  names.push_back("r_leg_aky");
  names.push_back("r_leg_akx");
  names.push_back("r_arm_mwx");
  names.push_back("neck_ay");
  return names;
}


QMap<QString, QString> PackageSearchPaths;

int feq (double a, double b)
{
    return fabs (a - b) < 1e-9;
}

void bot_quat_to_angle_axis (const double q[4], double *theta, double axis[3])
{
    double halftheta = acos (q[0]);
    *theta = halftheta * 2;
    double sinhalftheta = sin (halftheta);
    if (feq (halftheta, 0)) {
        axis[0] = 0;
        axis[1] = 0;
        axis[2] = 1;
        *theta = 0;
    } else {
        axis[0] = q[1] / sinhalftheta;
        axis[1] = q[2] / sinhalftheta;
        axis[2] = q[3] / sinhalftheta;
    }
}

void bot_quat_to_roll_pitch_yaw (const double q[4], double rpy[3])
{
    double roll_a = 2 * (q[0]*q[1] + q[2]*q[3]);
    double roll_b = 1 - 2 * (q[1]*q[1] + q[2]*q[2]);
    rpy[0] = atan2 (roll_a, roll_b);

    double pitch_sin = 2 * (q[0]*q[2] - q[3]*q[1]);
    rpy[1] = asin (pitch_sin);

    double yaw_a = 2 * (q[0]*q[3] + q[1]*q[2]);
    double yaw_b = 1 - 2 * (q[2]*q[2] + q[3]*q[3]);
    rpy[2] = atan2 (yaw_a, yaw_b);
}

vtkSmartPointer<vtkPolyData> shallowCopy(vtkPolyData* polyData)
{
  if (!polyData)
  {
    return 0;
  }

  vtkSmartPointer<vtkPolyData> newPolyData = vtkSmartPointer<vtkPolyData>::New();
  newPolyData->ShallowCopy(polyData);
  return newPolyData;
}

vtkSmartPointer<vtkPolyData> computeNormals(vtkPolyData* polyData)
{
  vtkSmartPointer<vtkPolyDataNormals> normalsFilter = vtkSmartPointer<vtkPolyDataNormals>::New();
  normalsFilter->SetFeatureAngle(45);
  SetInputData(normalsFilter, polyData);
  normalsFilter->Update();
  return shallowCopy(normalsFilter->GetOutput());
}

vtkSmartPointer<vtkPolyData> transformPolyData(vtkPolyData* polyData, vtkTransform* transform)
{
  vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
  transformFilter->SetTransform(transform);
  SetInputData(transformFilter, polyData);
  transformFilter->Update();
  return shallowCopy(transformFilter->GetOutput());
}

vtkSmartPointer<vtkPolyData> loadPolyData(const std::string filename)
{
  vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New();
  reader->SetFileName(filename.c_str());

  /*
  std::string chullFilename = filename;
  boost::algorithm::replace_last(chullFilename, ".obj", "_chull.obj");
  if (boost::filesystem::exists(chullFilename))
  {
    reader->SetFileName(chullFilename.c_str());
  }
  */

  reader->Update();
  if (!reader->GetOutput()->GetNumberOfPoints())
  {
    std::cout << "Failed to load data from: " << filename << std::endl;
    return 0;
  }

  return shallowCopy(reader->GetOutput());
}

/*
void QuaternionToAngleAxis(double wxyz[4], double angleAxis[4])
{
  vtkQuaternion<double> quat(wxyz);
  angleAxis[0] = quat.GetRotationAngleAndAxis(angleAxis+1);
  angleAxis[0] = vtkMath::DegreesFromRadians(angleAxis[0]);
}
*/

ddMeshVisual::Ptr visualFromPolyData(vtkSmartPointer<vtkPolyData> polyData)
{
  ddMeshVisual::Ptr visual(new ddMeshVisual);
  visual->PolyData = computeNormals(polyData);
  visual->Actor = vtkSmartPointer<vtkActor>::New();

  visual->Actor->GetProperty()->SetSpecular(0.9);
  visual->Actor->GetProperty()->SetSpecularPower(20);
  //visual->Actor->GetProperty()->SetColor(148/255.0, 147/255.0, 155/255.0);
  visual->Actor->GetProperty()->SetColor(190/255.0, 190/255.0, 190/255.0);
  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  SetInputData(mapper, visual->PolyData);
  visual->Actor->SetMapper(mapper);

  return visual;
}

ddMeshVisual::Ptr loadMeshVisual(const std::string& filename)
{
  vtkSmartPointer<vtkPolyData> polyData = loadPolyData(filename);
  if (!polyData)
  {
    return ddMeshVisual::Ptr();
  }

  return visualFromPolyData(polyData);
}

ddMeshVisual::Ptr makeSphereVisual(double radius)
{
  vtkSmartPointer<vtkSphereSource> sphere = vtkSmartPointer<vtkSphereSource>::New();
  sphere->SetPhiResolution(24);
  sphere->SetThetaResolution(24);
  sphere->SetRadius(radius);
  sphere->Update();

  return visualFromPolyData(shallowCopy(sphere->GetOutput()));
}

ddMeshVisual::Ptr makeCylinderVisual(double radius, double length)
{
  vtkSmartPointer<vtkCylinderSource> cylinder = vtkSmartPointer<vtkCylinderSource>::New();
  cylinder->SetHeight(length);
  cylinder->SetRadius(radius);
  cylinder->SetResolution(24);
  cylinder->Update();

  vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
  transform->RotateX(90);
  return visualFromPolyData(transformPolyData(cylinder->GetOutput(), transform));
}

ddMeshVisual::Ptr makeBoxVisual(double x, double y, double z)
{
  vtkSmartPointer<vtkCubeSource> cube = vtkSmartPointer<vtkCubeSource>::New();
  cube->SetXLength(x);
  cube->SetYLength(y);
  cube->SetZLength(z);
  cube->Update();
  return visualFromPolyData(shallowCopy(cube->GetOutput()));
}

class URDFRigidBodyManipulatorVTK : public URDFRigidBodyManipulator
{
public:

  typedef std::map<boost::shared_ptr<urdf::Visual>, ddMeshVisual::Ptr> MeshMapType;

  MeshMapType mesh_map;

  ddPtrMacro(URDFRigidBodyManipulatorVTK);

  URDFRigidBodyManipulatorVTK()
  {

  }

  virtual ~URDFRigidBodyManipulatorVTK()
  {

  }

  std::vector<ddMeshVisual::Ptr> meshVisuals()
  {
    std::vector<ddMeshVisual::Ptr> visuals;
    for (MeshMapType::iterator itr = mesh_map.begin(); itr != mesh_map.end(); ++itr)
    {
      visuals.push_back(itr->second);
    }
    return visuals;
  }

  std::string locateMeshFile(boost::shared_ptr<urdf::Mesh> mesh, std::string root_dir)
  {

    string fname = mesh->filename;
    bool has_package = boost::find_first(mesh->filename,"package://");

    if (has_package)
    {
      //cout << "replacing " << fname;
      boost::replace_first(fname,"package://","");
      string package = fname.substr(0,fname.find_first_of("/"));

      QString packageDir = ddDrakeModel::findPackageDirectory(package.c_str());
      if (packageDir.isEmpty())
      {
        std::cout << "Failed to locate package: " << package << std::endl;
        return std::string();
      }


      //boost::replace_first(fname,package,rospack(package));

      boost::replace_first(fname, package, packageDir.toAscii().data());
      //cout << " with " << fname << endl;
    }
    else
    {
      fname = root_dir + "/" + mesh->filename;
    }

    if (!boost::filesystem::exists(fname))
    {
      cerr << "cannot find mesh file: " << fname;
      if (has_package)
        cerr << " (note: original mesh string had a package:// in it, and I haven't really implemented rospack yet)";
      cerr << endl;
      return std::string();
    }

    boost::filesystem::path mypath(fname);
    string ext = mypath.extension().native();
    boost::to_lower(ext);

    if (mypath.extension().native() != ".obj")
    {
      std::string fnameAsObj = mypath.replace_extension(".obj").native();
      if ( boost::filesystem::exists( fnameAsObj ) )
      {
        return fnameAsObj;
      }
      else
      {
        cerr << "Warning: Mesh " << fname << " ignored because it does not have extension .obj (nor can I find a juxtaposed file with a .obj extension)" << endl;
        return std::string();
      }
    }
    else
    {
      return fname;
    }
  }


  virtual bool addURDF(boost::shared_ptr<urdf::ModelInterface> _urdf_model,
                       std::map<std::string, int> jointname_to_jointnum,
                       std::map<std::string,int> dofname_to_dofnum,
                       const std::string & root_dir=".")
  {
    if (!URDFRigidBodyManipulator::addURDF(_urdf_model, jointname_to_jointnum, dofname_to_dofnum, root_dir))
      return false;

    for (map<string, boost::shared_ptr<urdf::Link> >::iterator l=_urdf_model->links_.begin(); l!=_urdf_model->links_.end(); l++)
    {
      // load geometry
      if (l->second->visual) // then at least one default visual tag exists
      {

        // todo: iterate over all visual groups (not just "default")

        map<string, boost::shared_ptr<vector<boost::shared_ptr<urdf::Visual> > > >::iterator v_grp_it = l->second->visual_groups.find("default");
        vector<boost::shared_ptr<urdf::Visual> > visuals = (*v_grp_it->second);

        for (size_t iv = 0;iv < v_grp_it->second->size();iv++)
        {

          boost::shared_ptr<urdf::Visual> vptr = visuals[iv];

          int visualType = vptr->geometry->type;
          ddMeshVisual::Ptr meshVisual;

          if (visualType == urdf::Geometry::MESH)
          {
            boost::shared_ptr<urdf::Mesh> mesh(boost::dynamic_pointer_cast<urdf::Mesh>(vptr->geometry));

            std::string filename = locateMeshFile(mesh, root_dir);
            if (filename.size())
            {
              meshVisual = loadMeshVisual(filename);
            }
          }
          else if (visualType == urdf::Geometry::SPHERE)
          {
            boost::shared_ptr<urdf::Sphere> sphere(boost::dynamic_pointer_cast<urdf::Sphere>(vptr->geometry));
            double radius = sphere->radius;
            meshVisual =  makeSphereVisual(radius);
          }
          else if (visualType == urdf::Geometry::BOX)
          {
            boost::shared_ptr<urdf::Box> box(boost::dynamic_pointer_cast<urdf::Box>(vptr->geometry));
            meshVisual =  makeBoxVisual(box->dim.x, box->dim.y, box->dim.z);
          }
          else if (visualType == urdf::Geometry::CYLINDER)
          {
            boost::shared_ptr<urdf::Cylinder> cyl(boost::dynamic_pointer_cast<urdf::Cylinder>(vptr->geometry));
            meshVisual =  makeCylinderVisual(cyl->radius, cyl->length);
          }

          if (meshVisual)
          {
            meshVisual->Name = l->second->name;
            mesh_map[vptr] = meshVisual;
          }


        }
      }
    }


    return true;
  }


  virtual void updateModel()
  {
    const Vector4d zero(0,0,0,1);
    double theta, axis[3], quat[4];
    double angleAxis[4];
    Matrix<double,7,1> pose;

    // iterate over each model
    for (int robot=0; robot< urdf_model.size(); robot++)
    {

      // iterate over each link and draw
      for (map<string, boost::shared_ptr<urdf::Link> >::iterator l=urdf_model[robot]->links_.begin(); l!=urdf_model[robot]->links_.end(); l++)
      {
        if (!l->second->visual) // then at least one default visual tag exists
        {
          continue;
        }

        int body_ind;
        if (l->second->parent_joint)
        {
          map<string, int>::const_iterator j2 = findWithSuffix(joint_map[robot],l->second->parent_joint->name);
          if (j2 == joint_map[robot].end()) continue;  // this shouldn't happen, but just in case...
          body_ind = j2->second;
        }
        else
        {
          map<string, int>::const_iterator j2 = findWithSuffix(joint_map[robot],"base");
          if (j2 == joint_map[robot].end()) continue;  // this shouldn't happen, but just in case...
          body_ind = j2->second;  // then it's attached directly to the floating base
        }

        //cout << "drawing robot " << robot << " body_ind " << body_ind << ": " << bodies[body_ind].linkname << endl;

        forwardKin(body_ind, zero, 2, pose);

        //cout << l->second->name << " is at " << pose.transpose() << endl;

        double* posedata = pose.data();

        //bot_quat_to_angle_axis(&posedata[3], &theta, axis);
        //glPushMatrix();
        //glTranslatef(pose(0),pose(1),pose(2));
        //glRotatef(theta * 180/3.141592654, axis[0], axis[1], axis[2]);



        //QuaternionToAngleAxis(&posedata[3], angleAxis);
        bot_quat_to_angle_axis(&posedata[3], &angleAxis[0], &angleAxis[1]);
        angleAxis[0] = vtkMath::DegreesFromRadians(angleAxis[0]);

        vtkSmartPointer<vtkTransform> worldToLink = vtkSmartPointer<vtkTransform>::New();
        worldToLink->PreMultiply();
        worldToLink->Translate(pose(0), pose(1), pose(2));
        worldToLink->RotateWXYZ(angleAxis[0], angleAxis[1], angleAxis[2], angleAxis[3]);

        //printf("link rotation: [%f, %f, %f, %f]\n", angleAxis[0], angleAxis[1], angleAxis[2], angleAxis[3]);
        //printf("link translation: [%f, %f, %f]\n", pose(0), pose(1), pose(2));


        // todo: iterate over all visual groups (not just "default")
        map<string, boost::shared_ptr<vector<boost::shared_ptr<urdf::Visual> > > >::iterator v_grp_it = l->second->visual_groups.find("default");
        if (v_grp_it == l->second->visual_groups.end()) continue;

        vector<boost::shared_ptr<urdf::Visual> > *visuals = v_grp_it->second.get();
        for (vector<boost::shared_ptr<urdf::Visual> >::iterator viter = visuals->begin(); viter!=visuals->end(); viter++)
        {
          boost::shared_ptr<urdf::Visual> vptr = *viter;
          if (!vptr) continue;

          //glPushMatrix();

          // handle visual material
          if (vptr->material)
          {
            //glColor4f(vptr->material->color.r,
            //    vptr->material->color.g,
            //    vptr->material->color.b,
            //    vptr->material->color.a);
          }

          // todo: handle textures here?

          // handle visual origin

          /*
          glTranslatef(vptr->origin.position.x,
                       vptr->origin.position.y,
                       vptr->origin.position.z);

          quat[0] = vptr->origin.rotation.w;
          quat[1] = vptr->origin.rotation.x;
          quat[2] = vptr->origin.rotation.y;
          quat[3] = vptr->origin.rotation.z;
          bot_quat_to_angle_axis(quat, &theta, axis);
          glRotatef(theta * 180/3.141592654, axis[0], axis[1], axis[2]);
          */

          quat[0] = vptr->origin.rotation.w;
          quat[1] = vptr->origin.rotation.x;
          quat[2] = vptr->origin.rotation.y;
          quat[3] = vptr->origin.rotation.z;

          //QuaternionToAngleAxis(quat, angleAxis);
          bot_quat_to_angle_axis(quat, &angleAxis[0], &angleAxis[1]);
          angleAxis[0] = vtkMath::DegreesFromRadians(angleAxis[0]);

          vtkSmartPointer<vtkTransform> linkToVisual = vtkSmartPointer<vtkTransform>::New();
          linkToVisual->PreMultiply();
          linkToVisual->Translate(vptr->origin.position.x,
                                 vptr->origin.position.y,
                                 vptr->origin.position.z);
          linkToVisual->RotateWXYZ(angleAxis[0], angleAxis[1], angleAxis[2], angleAxis[3]);

          //printf("visual rotation: [%f, %f, %f, %f]\n", angleAxis[0], angleAxis[1], angleAxis[2], angleAxis[3]);
          //printf("visual translation: [%f, %f, %f]\n", vptr->origin.position.x, vptr->origin.position.y, vptr->origin.position.z);


          int visualType = vptr->geometry->type;
          if (visualType == urdf::Geometry::MESH)
          {

            boost::shared_ptr<urdf::Mesh> mesh(boost::dynamic_pointer_cast<urdf::Mesh>(vptr->geometry));

            //glScalef(mesh->scale.x,mesh->scale.y,mesh->scale.z);

            linkToVisual->Scale(mesh->scale.x,
                                mesh->scale.y,
                                mesh->scale.z);

            //printf("visual scale: [%f, %f, %f]\n", mesh->scale.x, mesh->scale.y, mesh->scale.z);

          }

          vtkSmartPointer<vtkTransform> worldToVisual = vtkSmartPointer<vtkTransform>::New();
          worldToVisual->PreMultiply();
          worldToVisual->Concatenate(worldToLink);
          worldToVisual->Concatenate(linkToVisual);
          worldToVisual->Update();

          MeshMapType::iterator iter = mesh_map.find(vptr);
          if (iter!= mesh_map.end())
          {
            ddMeshVisual::Ptr meshVisual = iter->second;
            meshVisual->Actor->SetUserTransform(worldToVisual);

          }

        } // end loop over visuals
      } // end loop over links
    } // end loop over robots
  }


  virtual QMap<QString, int> getLinkIds()
  {
    QMap<QString, int> linkMap;

    // iterate over each model
    for (int robot=0; robot< urdf_model.size(); robot++)
    {
      // iterate over each link
      for (map<string, boost::shared_ptr<urdf::Link> >::iterator l=urdf_model[robot]->links_.begin(); l!=urdf_model[robot]->links_.end(); l++)
      {


        int body_ind;
        if (l->second->parent_joint)
        {
          map<string, int>::const_iterator j2 = findWithSuffix(joint_map[robot],l->second->parent_joint->name);
          if (j2 == joint_map[robot].end()) continue;  // this shouldn't happen, but just in case...
          body_ind = j2->second;
        }
        else
        {
          map<string, int>::const_iterator j2 = findWithSuffix(joint_map[robot],"base");
          if (j2 == joint_map[robot].end()) continue;  // this shouldn't happen, but just in case...
          body_ind = j2->second;  // then it's attached directly to the floating base
        }

        linkMap[l->second->name.c_str()] = body_ind;
      }
    }

    return linkMap;

  }

  vtkSmartPointer<vtkTransform> getLinkToWorld(QString linkName)
  {
    QMap<QString, int> linkMap = this->getLinkIds();

    if (!linkMap.contains(linkName))
    {
      printf("getLinkToWorld: cannot find link name: %s\n", linkName.toAscii().data());
      return NULL;
    }


    const Vector4d zero(0,0,0,1);
    double theta, axis[3], quat[4];
    double angleAxis[4];
    Matrix<double, 7 ,1> pose;

    forwardKin(linkMap.value(linkName), zero, 2, pose);

    double* posedata = pose.data();
    bot_quat_to_angle_axis(&posedata[3], &angleAxis[0], &angleAxis[1]);
    vtkSmartPointer<vtkTransform> linkToWorld = vtkSmartPointer<vtkTransform>::New();
    linkToWorld->PostMultiply();
    linkToWorld->RotateWXYZ(vtkMath::DegreesFromRadians(angleAxis[0]), angleAxis[1], angleAxis[2], angleAxis[3]);
    linkToWorld->Translate(pose(0), pose(1), pose(2));
    return linkToWorld;
  }

};


URDFRigidBodyManipulatorVTK::Ptr loadVTKModelFromFile(const string &urdf_filename)
{
  // urdf_filename can be a list of urdf files seperated by a :

  URDFRigidBodyManipulatorVTK::Ptr model(new URDFRigidBodyManipulatorVTK);

  string token;
  istringstream iss(urdf_filename);

  while (getline(iss,token,':'))
  {
    fstream xml_file(token.c_str(), fstream::in);
      string xml_string;
    if (xml_file.is_open())
    {
        while ( xml_file.good() )
      {
          string line;
          getline( xml_file, line);
          xml_string += (line + "\n");
        }
        xml_file.close();
    }
    else
    {
        cerr << "Could not open file ["<<urdf_filename.c_str()<<"] for parsing."<< endl;
        return URDFRigidBodyManipulatorVTK::Ptr();
    }

    string pathname;
    boost::filesystem::path mypath(urdf_filename);
    if (!mypath.empty() && mypath.has_parent_path())  
      pathname = mypath.parent_path().native();

    // parse URDF to get model
    model->addURDFfromXML(xml_string,pathname);
  }

  return model;
}


URDFRigidBodyManipulatorVTK::Ptr loadVTKModelFromXML(const string &xmlString)
{
  URDFRigidBodyManipulatorVTK::Ptr model(new URDFRigidBodyManipulatorVTK);
  model->addURDFfromXML(xmlString, "");
  return model;
}


} // end namespace

//-----------------------------------------------------------------------------
class ddDrakeModel::ddInternal
{
public:

  ddInternal()
  {
    this->Visible = true;
    this->Alpha = 1.0;
    this->Color = Qt::white;
  }

  URDFRigidBodyManipulatorVTK::Ptr Model;

  QString FileName;
  bool Visible;
  double Alpha;
  QColor Color;
};


//-----------------------------------------------------------------------------
ddDrakeModel::ddDrakeModel(QObject* parent) : QObject(parent)
{
  this->Internal = new ddInternal;
}

//-----------------------------------------------------------------------------
ddDrakeModel::~ddDrakeModel()
{
  delete this->Internal;
}

//-----------------------------------------------------------------------------
int ddDrakeModel::numberOfJoints()
{
  if (!this->Internal->Model)
  {
    return 0;
  }

  return this->Internal->Model->num_dof;
}

//-----------------------------------------------------------------------------
void ddDrakeModel::setJointPositions(const QList<double>& jointPositions)
{
  URDFRigidBodyManipulatorVTK::Ptr model = this->Internal->Model;

  if (!model)
  {
    std::cout << "ddDrakeModel::setJointPositions(): model is null" << std::endl;
    return;
  }

  const std::map<std::string, int> dofMap = model->dof_map[0];
  std::vector<std::string> dofNames = getDofNames();

  if (jointPositions.length() != dofNames.size())
  {
    std::cout << "ddDrakeModel::setJointPositions(): input jointPositions length "
              << jointPositions.length() << " != " << model->num_dof << std::endl;
    return;
  }

  MatrixXd q = MatrixXd::Zero(model->num_dof, 1);
  for (int i = 0; i < dofNames.size(); ++i)
  {
    const std::string& dofName = dofNames[i];

    std::map<std::string, int>::const_iterator itr = dofMap.find(dofName);
    if (itr == dofMap.end())
    {
      printf("Could not find URDF model dof with name: %s\n", dofName.c_str());
    }

    int dofId = itr->second;
    q(dofId, 0) = jointPositions[i];
    //printf("rbm dof %02d %s  -->  %d\n", i, dofName.c_str(), dofId);
  }

  model->doKinematics(q.data());
  model->updateModel();
  emit this->modelChanged();
}


//-----------------------------------------------------------------------------
bool ddDrakeModel::getLinkToWorld(const QString& linkName, vtkTransform* transform)
{
  if (!transform || !this->Internal->Model)
  {
    return false;
  }

  vtkSmartPointer<vtkTransform> linkToWorld = this->Internal->Model->getLinkToWorld(linkName);
  if (linkToWorld)
  {
    transform->SetMatrix(linkToWorld->GetMatrix());
    return true;
  }
  return false;
}

//-----------------------------------------------------------------------------
QList<QString> ddDrakeModel::getLinkNames()
{
  if (!this->Internal->Model)
  {
    return QList<QString>();
  }
  return this->Internal->Model->getLinkIds().keys();
}

//-----------------------------------------------------------------------------
QString ddDrakeModel::getLinkNameForMesh(vtkPolyData* polyData)
{
  std::vector<ddMeshVisual::Ptr> visuals = this->Internal->Model->meshVisuals();
  for (size_t i = 0; i < visuals.size(); ++i)
  {
    if (visuals[i]->PolyData == polyData)
    {
      return visuals[i]->Name.c_str();
    }
  }
  return QString();
}


//-----------------------------------------------------------------------------
void ddDrakeModel::setEstRobotState(const QList<double>& robotState)
{
  URDFRigidBodyManipulatorVTK::Ptr model = this->Internal->Model;

  if (!model)
  {
    std::cout << "ddDrakeModel::setJointPositions(): model is null" << std::endl;
    return;
  }

  const std::map<std::string, int> dofMap = model->dof_map[0];
  std::vector<std::string> dofNames = getEstRobotStateJointNames();

  MatrixXd q = MatrixXd::Zero(model->num_dof, 1);
  for (int i = 0; i < dofNames.size(); ++i)
  {
    const std::string& dofName = dofNames[i];

    std::map<std::string, int>::const_iterator itr = dofMap.find(dofName);
    if (itr == dofMap.end())
    {
      continue;
    }

    int dofId = itr->second;

    //printf("est dof %02d %s  -->  %d\n", i, dofName.c_str(), dofId);

    q(dofId, 0) = robotState[7 + i];
  }

  q(dofMap.find("base_x")->second, 0) = robotState[0];
  q(dofMap.find("base_y")->second, 0) = robotState[1];
  q(dofMap.find("base_z")->second, 0) = robotState[2];

  double base_rpy[3];
  double base_quat[4] = {robotState[3], robotState[4], robotState[5], robotState[6]};


  bot_quat_to_roll_pitch_yaw(base_quat, base_rpy);

  q(dofMap.find("base_roll")->second, 0) = base_rpy[0];
  q(dofMap.find("base_pitch")->second, 0) = base_rpy[1];
  q(dofMap.find("base_yaw")->second, 0) = base_rpy[2];


  model->doKinematics(q.data());
  model->updateModel();
  emit this->modelChanged();
}

//-----------------------------------------------------------------------------
bool ddDrakeModel::loadFromFile(const QString& filename)
{
  URDFRigidBodyManipulatorVTK::Ptr model = loadVTKModelFromFile(filename.toAscii().data());
  if (!model)
  {
    return false;
  }

  this->Internal->FileName = filename;
  this->Internal->Model = model;

  MatrixXd q0 = MatrixXd::Zero(model->num_dof, 1);
  model->doKinematics(q0.data());
  model->updateModel();
  return true;
}

//-----------------------------------------------------------------------------
bool ddDrakeModel::loadFromXML(const QString& xmlString)
{
  URDFRigidBodyManipulatorVTK::Ptr model = loadVTKModelFromXML(xmlString.toAscii().data());
  if (!model)
  {
    return false;
  }

  this->Internal->FileName = "<xml string>";
  this->Internal->Model = model;

  MatrixXd q0 = MatrixXd::Zero(model->num_dof, 1);
  model->doKinematics(q0.data());
  model->updateModel();
  return true;
}

//-----------------------------------------------------------------------------
void ddDrakeModel::addToRenderer(vtkRenderer* renderer)
{
  if (!renderer)
  {
    return;
  }

  std::vector<ddMeshVisual::Ptr> visuals = this->Internal->Model->meshVisuals();

  for (size_t i = 0; i < visuals.size(); ++i)
  {
    renderer->AddActor(visuals[i]->Actor);
  }

  renderer->ResetCamera();
}

//-----------------------------------------------------------------------------
void ddDrakeModel::removeFromRenderer(vtkRenderer* renderer)
{
  if (!renderer)
  {
    return;
  }

  std::vector<ddMeshVisual::Ptr> visuals = this->Internal->Model->meshVisuals();

  for (size_t i = 0; i < visuals.size(); ++i)
  {
    renderer->RemoveActor(visuals[i]->Actor);
  }

  renderer->ResetCamera();
}

//-----------------------------------------------------------------------------
const QString& ddDrakeModel::filename() const
{
  return this->Internal->FileName;
}

//-----------------------------------------------------------------------------
QColor ddDrakeModel::color() const
{
  return this->Internal->Color;
}

//-----------------------------------------------------------------------------
void ddDrakeModel::setColor(const QColor& color)
{
  this->Internal->Color = color;

  double red = color.redF();
  double green = color.greenF();
  double blue = color.blueF();
  double alpha = color.alphaF();

  std::vector<ddMeshVisual::Ptr> visuals = this->Internal->Model->meshVisuals();
  for (size_t i = 0; i < visuals.size(); ++i)
  {
    visuals[i]->Actor->GetProperty()->SetColor(red, green, blue);
  }
  this->setAlpha(alpha);
  emit this->modelChanged();
}

//-----------------------------------------------------------------------------
void ddDrakeModel::setLinkColor(const QString& linkName, const QColor& color)
{
  double red = color.redF();
  double green = color.greenF();
  double blue = color.blueF();
  double alpha = color.alphaF();


  std::vector<ddMeshVisual::Ptr> visuals = this->Internal->Model->meshVisuals();
  for (size_t i = 0; i < visuals.size(); ++i)
  {
    if (visuals[i]->Name == linkName.toAscii().data())
    {
      visuals[i]->Actor->GetProperty()->SetColor(red, green, blue);
      visuals[i]->Actor->GetProperty()->SetOpacity(alpha);
    }
  }
  emit this->modelChanged();
}

//-----------------------------------------------------------------------------
double ddDrakeModel::alpha() const
{
  return this->Internal->Alpha;
}

//-----------------------------------------------------------------------------
void ddDrakeModel::setAlpha(double alpha)
{
  std::vector<ddMeshVisual::Ptr> visuals = this->Internal->Model->meshVisuals();
  for (size_t i = 0; i < visuals.size(); ++i)
  {
    visuals[i]->Actor->GetProperty()->SetOpacity(alpha);
  }
  this->Internal->Alpha = alpha;
  emit this->modelChanged();
}

//-----------------------------------------------------------------------------
void ddDrakeModel::setVisible(bool visible)
{
  std::vector<ddMeshVisual::Ptr> visuals = this->Internal->Model->meshVisuals();

  for (size_t i = 0; i < visuals.size(); ++i)
  {
    visuals[i]->Actor->SetVisibility(visible);
  }
  this->Internal->Visible = visible;
  emit this->modelChanged();
}

//-----------------------------------------------------------------------------
bool ddDrakeModel::visible() const
{
  return this->Internal->Visible;
}

//-----------------------------------------------------------------------------
void ddDrakeModel::addPackageSearchPath(const QString& searchPath)
{
  PackageSearchPaths[QDir(searchPath).dirName()] = searchPath;
}

//-----------------------------------------------------------------------------
QString ddDrakeModel::findPackageDirectory(const QString& packageName)
{
  return PackageSearchPaths.value(packageName);
}
