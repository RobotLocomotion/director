#include "ddDrakeModel.h"

#include <URDFRigidBodyManipulator.h>

#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

#include <map>
#include <vector>
#include <string>
#include <sstream>
#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>

#include <urdf_interface/model.h>

using std::map;
using std::vector;
using std::string;
using std::istringstream;

namespace
{

vtkSmartPointer<vtkPolyData> loadMesh(const std::string filename)
{
  return vtkSmartPointer<vtkPolyData>::New();
}

class URDFRigidBodyManipulatorVTK : public URDFRigidBodyManipulator
{
public:

  map<string, vtkSmartPointer<vtkPolyData> > mesh_map;

  URDFRigidBodyManipulatorVTK()
  {

  }

  virtual ~URDFRigidBodyManipulatorVTK()
  {

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

            map<string, vtkSmartPointer<vtkPolyData> >::iterator iter = mesh_map.find(mesh->filename);
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
              cout << "Loading mesh: " << fname << endl;
              vtkSmartPointer<vtkPolyData> polyData = loadMesh(fname);

              if (!polyData)
              {
                cerr << "Error loading mesh: " << fname << endl;
              }
              else
              {
                mesh_map.insert(make_pair(mesh->filename, polyData));
              }

            }
            else
            {
              // try changing the extension to dae and loading

              fname = mypath.replace_extension(".obj").native();

              if ( boost::filesystem::exists( fname ) )
              {
                cout << "Loading mesh: " << fname << endl;
                vtkSmartPointer<vtkPolyData> polyData = loadMesh(fname);

                if (!polyData)
                {
                  cerr << "Error loading mesh: " << fname << endl;
                }
                else
                {
                  mesh_map.insert(make_pair(mesh->filename, polyData));
                }

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

          cout << "drawing robot " << robot << " body_ind " << body_ind << ": " << bodies[body_ind].linkname << endl;

          forwardKin(body_ind,zero,2,pose);

          cout << l->second->name << " is at " << pose.transpose() << endl;

          double* posedata = pose.data();

          //bot_quat_to_angle_axis(&posedata[3], &theta, axis);
          //glPushMatrix();
          //glTranslatef(pose(0),pose(1),pose(2));
          //glRotatef(theta * 180/3.141592654, axis[0], axis[1], axis[2]);

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

              map<string,vtkSmartPointer<vtkPolyData> >::iterator iter = mesh_map.find(mesh->filename);
              if (iter!= mesh_map.end())
              {
                //bot_wavefront_model_gl_draw(iter->second);
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


URDFRigidBodyManipulatorVTK* loadVTKModelFromFile(const string &urdf_filename)
{
  // urdf_filename can be a list of urdf files seperated by a :

  URDFRigidBodyManipulatorVTK* model = new URDFRigidBodyManipulatorVTK();


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
        return NULL;
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

  URDFRigidBodyManipulator* Model;

};


//-----------------------------------------------------------------------------
ddDrakeModel::ddDrakeModel(QObject* parent) : QObject(parent)
{
  this->Internal = new ddInternal;

  QString modelFile = "/source/drc/drc-trunk/software/models/mit_gazebo_models/mit_robot_drake/model.urdf";
  URDFRigidBodyManipulatorVTK* model = loadVTKModelFromFile(modelFile.toAscii().data());

  model->updateModel();
}

//-----------------------------------------------------------------------------
ddDrakeModel::~ddDrakeModel()
{
  delete this->Internal;
}
