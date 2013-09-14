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
#include <vtkTransform.h>
#include <vtkQuaternion.h>
#include <vtkProperty.h>

#include <map>
#include <vector>
#include <string>
#include <sstream>
#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>

#include <urdf_interface/model.h>

#include <math.h>

using std::map;
using std::vector;
using std::string;
using std::istringstream;

class ddMeshVisual
{
 public:

  ddPtrMacro(ddMeshVisual);
  ddMeshVisual()
  {

  }

  std::string FileName;
  vtkSmartPointer<vtkPolyData> PolyData;
  vtkSmartPointer<vtkActor> Actor;
  vtkSmartPointer<vtkTransform> Transform;

private:

  Q_DISABLE_COPY(ddMeshVisual);
};

namespace
{

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
  normalsFilter->SetInputData(polyData);
  normalsFilter->Update();
  return shallowCopy(normalsFilter->GetOutput());
}

vtkSmartPointer<vtkPolyData> loadPolyData(const std::string filename)
{
  vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New();
  reader->SetFileName(filename.c_str());
  reader->Update();
  if (!reader->GetOutput()->GetNumberOfPoints())
  {
    std::cout << "Failed to load data from: " << filename << std::endl;
  }

  return shallowCopy(reader->GetOutput());
}

void QuaternionToAngleAxis(double wxyz[4], double angleAxis[4])
{
  vtkQuaternion<double> quat(wxyz);
  angleAxis[0] = quat.GetRotationAngleAndAxis(angleAxis+1);
  angleAxis[0] = vtkMath::DegreesFromRadians(angleAxis[0]);
}

ddMeshVisual::Ptr loadMeshVisual(const std::string& filename)
{
  ddMeshVisual::Ptr mesh(new ddMeshVisual);
  mesh->FileName = filename;
  mesh->PolyData = computeNormals(loadPolyData(filename));
  mesh->Actor = vtkSmartPointer<vtkActor>::New();

  mesh->Actor->GetProperty()->SetSpecular(0.9);
  mesh->Actor->GetProperty()->SetSpecularPower(20);
  //mesh->Actor->GetProperty()->SetColor(148/255.0, 147/255.0, 155/255.0);
  mesh->Actor->GetProperty()->SetColor(190/255.0, 190/255.0, 190/255.0);
  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputData(mesh->PolyData);
  mesh->Actor->SetMapper(mapper);

  return mesh;
}

class URDFRigidBodyManipulatorVTK : public URDFRigidBodyManipulator
{
public:

  typedef std::map<string, ddMeshVisual::Ptr> MeshMapType;

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


  virtual void loadURDFMesh(boost::shared_ptr<urdf::Mesh> mesh, const std::string& filename)
  {
    cout << "Loading mesh: " << filename << endl;
    ddMeshVisual::Ptr meshVisual = loadMeshVisual(filename);

    if (!meshVisual)
    {
      cerr << "Error loading mesh from file: " << filename << endl;
    }
    else
    {
      mesh_map.insert(make_pair(mesh->filename, meshVisual));
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
        for (size_t iv = 0;iv < v_grp_it->second->size();iv++)
        {
          vector<boost::shared_ptr<urdf::Visual> > visuals = (*v_grp_it->second);

          if (visuals[iv]->geometry->type == urdf::Geometry::MESH)
          {
            boost::shared_ptr<urdf::Mesh> mesh(boost::dynamic_pointer_cast<urdf::Mesh>(visuals[iv]->geometry));

            MeshMapType::iterator iter = mesh_map.find(mesh->filename);
            if (iter != mesh_map.end())  // then it's already in the map... no need to load it again
              continue;

            string fname = mesh->filename;
            bool has_package = boost::find_first(mesh->filename,"package://");

            if (has_package)
            {
              cout << "replacing " << fname;
              boost::replace_first(fname,"package://","");
              string package = fname.substr(0,fname.find_first_of("/"));
              boost::replace_first(fname,package,rospack(package));
              cout << " with " << fname << endl;
            }
            else
            {
              fname = root_dir + "/" + mesh->filename;
            }
            boost::filesystem::path mypath(fname);

            if (!boost::filesystem::exists(fname))
            {
              cerr << "cannot find mesh file: " << fname;
              if (has_package)
                cerr << " (note: original mesh string had a package:// in it, and I haven't really implemented rospack yet)";
              cerr << endl;
              continue;
            }

            string ext = mypath.extension().native();
            boost::to_lower(ext);

            if (ext.compare(".obj") == 0)
            {
              loadURDFMesh(mesh, fname);
            }
            else
            {
              // try changing the extension to dae and loading

              fname = mypath.replace_extension(".obj").native();

              if ( boost::filesystem::exists( fname ) )
              {
                loadURDFMesh(mesh, fname);
              }
              else
              {
                cerr << "Warning: Mesh " << fname << " ignored because it does not have extension .obj (nor can I find a juxtaposed file with a .obj extension)" << endl;
              }
            }
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
        if (l->second->visual) // then at least one default visual tag exists
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

          //cout << "drawing robot " << robot << " body_ind " << body_ind << ": " << bodies[body_ind].linkname << endl;

          forwardKin(body_ind,zero,2,pose);

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
            urdf::Visual * vptr = viter->get();
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


            int type = vptr->geometry->type;
            if (type == urdf::Geometry::SPHERE)
            {
              boost::shared_ptr<urdf::Sphere> sphere(boost::dynamic_pointer_cast<urdf::Sphere>(vptr->geometry));
              double radius = sphere->radius;

              //glutSolidSphere(radius,36,36);
            }
            else if (type == urdf::Geometry::BOX)
            {

              boost::shared_ptr<urdf::Box> box(boost::dynamic_pointer_cast<urdf::Box>(vptr->geometry));

              //glScalef(box->dim.x,box->dim.y,box->dim.z);
              //bot_gl_draw_cube();
            }
            else if (type == urdf::Geometry::CYLINDER)
            {

              boost::shared_ptr<urdf::Cylinder> cyl(boost::dynamic_pointer_cast<urdf::Cylinder>(vptr->geometry));

              // transform to center of cylinder
              /*
              glTranslatef(0.0,0.0,-cyl->length/2.0);
              GLUquadricObj* quadric = gluNewQuadric();
              gluQuadricDrawStyle(quadric, GLU_FILL);
              gluQuadricNormals(quadric, GLU_SMOOTH);
              gluQuadricOrientation(quadric, GLU_OUTSIDE);
              gluCylinder(quadric,
                    cyl->radius,
                    cyl->radius,
                    (double) cyl->length,
                    36,
                    1);
              gluDeleteQuadric(quadric);
              */

            }
            else if (type == urdf::Geometry::MESH)
            {

              boost::shared_ptr<urdf::Mesh> mesh(boost::dynamic_pointer_cast<urdf::Mesh>(vptr->geometry));

              //glScalef(mesh->scale.x,mesh->scale.y,mesh->scale.z);

              linkToVisual->Scale(mesh->scale.x,
                                  mesh->scale.y,
                                  mesh->scale.z);

              //printf("visual scale: [%f, %f, %f]\n", mesh->scale.x, mesh->scale.y, mesh->scale.z);

              vtkSmartPointer<vtkTransform> worldToVisual = vtkSmartPointer<vtkTransform>::New();
              worldToVisual->PreMultiply();
              worldToVisual->Concatenate(worldToLink);
              worldToVisual->Concatenate(linkToVisual);
              worldToVisual->Update();

              MeshMapType::iterator iter = mesh_map.find(mesh->filename);
              if (iter!= mesh_map.end())
              {
                //bot_wavefront_model_gl_draw(iter->second);

                ddMeshVisual::Ptr meshVisual = iter->second;
                if (meshVisual)
                {
                  //worldToVisual->Print(std::cout);
                  meshVisual->Actor->SetUserTransform(worldToVisual);
                }

              }

            }

            //glPopMatrix();
          } // end for loop over visuals

          //glPopMatrix();
        }
      } // end for loop over links
    }
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


} // end namespace

//-----------------------------------------------------------------------------
class ddDrakeModel::ddInternal
{
public:

  URDFRigidBodyManipulatorVTK::Ptr Model;
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
    return;
  }

  if (jointPositions.length() != model->num_dof)
  {
    return;
  }

  MatrixXd q = MatrixXd::Zero(model->num_dof, 1);
  for (int i = 0; i < model->num_dof; ++i)
  {
    q(i, 0) = jointPositions[i];
  }

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

  this->Internal->Model = model;

  MatrixXd q0 = MatrixXd::Zero(model->num_dof, 1);
  model->doKinematics(q0.data());
  model->updateModel();
  return true;
}

//-----------------------------------------------------------------------------
void ddDrakeModel::setAlpha(double alpha)
{
  std::vector<ddMeshVisual::Ptr> visuals = this->Internal->Model->meshVisuals();
  for (size_t i = 0; i < visuals.size(); ++i)
  {
    visuals[i]->Actor->GetProperty()->SetOpacity(alpha);
  }

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
