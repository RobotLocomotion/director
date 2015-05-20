#include "ddDrakeModel.h"
#include "ddSharedPtr.h"

#include <RigidBodyManipulator.h>
#include <shapes/Geometry.h>

#include <vtkPolyData.h>
#include <vtkAppendPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataNormals.h>
#include <vtkRenderer.h>
#include <vtkOBJReader.h>
#include <vtkSTLReader.h>
#include <vtkSphereSource.h>
#include <vtkCylinderSource.h>
#include <vtkCubeSource.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkTransform.h>
#include <vtkStringArray.h>
#include <vtkFieldData.h>
#include <vtkMath.h>
//#include <vtkQuaternion.h>
#include <vtkProperty.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLMultiBlockDataReader.h>
#include <vtkMultiBlockDataSet.h>
#include <vtkJPEGReader.h>
#include <vtkPNGReader.h>
#include <vtkImageData.h>

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

#if VTK_MAJOR_VERSION == 6
 #define SetInputData(filter, obj) filter->SetInputData(obj);
 #define AddInputData(filter, obj) filter->AddInputData(obj);
#else
  #define SetInputData(filter, obj) filter->SetInput(obj);
  #define AddInputData(filter, obj) filter->AddInput(obj);
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
  vtkSmartPointer<vtkActor> ShadowActor;
  vtkSmartPointer<vtkTransform> Transform;
  vtkSmartPointer<vtkTransform> VisualToLink;
  vtkSmartPointer<vtkTexture> Texture;
  std::string Name;

private:

  Q_DISABLE_COPY(ddMeshVisual);
};

namespace
{

const int GRAY_DEFAULT = 190;

std::map<std::string, std::string> PackageSearchPaths;

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

vtkSmartPointer<vtkTransform> makeTransform( const Eigen::Matrix4d& mat)
{
  vtkSmartPointer<vtkMatrix4x4> vtkmat = vtkSmartPointer<vtkMatrix4x4>::New();
  for (int i = 0; i < 4; ++i)
    {
    for (int j = 0; j < 4; ++j)
      {
      vtkmat->SetElement(i, j, mat(i,j));
      }
    }

  vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
  transform->SetMatrix(vtkmat);
  return transform;
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

bool endsWith(std::string const &fullString, std::string const &ending)
{
  if (fullString.length() >= ending.length())
  {
    return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
  } else
  {
    return false;
  }
}

vtkSmartPointer<vtkImageData> loadImage(const std::string& filename)
{
  vtkSmartPointer<vtkImageData> image;

  if (endsWith(boost::to_lower_copy(filename), "jpg"))
  {
    vtkSmartPointer<vtkJPEGReader> reader = vtkSmartPointer<vtkJPEGReader>::New();
    reader->SetFileName(filename.c_str());
    reader->Update();
    image = reader->GetOutput();
  }
  else if (endsWith(boost::to_lower_copy(filename), "png"))
  {
    vtkSmartPointer<vtkPNGReader> reader = vtkSmartPointer<vtkPNGReader>::New();
    reader->SetFileName(filename.c_str());
    reader->Update();
    image = reader->GetOutput();
  }

  if (!image->GetNumberOfPoints())
  {
    std::cout << "Failed to load data from: " << filename << std::endl;
    return 0;
  }

  return image;

}

std::vector<vtkSmartPointer<vtkPolyData> > loadPolyData(const std::string& filename)
{
  std::vector<vtkSmartPointer<vtkPolyData> > polyDataList;


  if (endsWith(boost::to_lower_copy(filename), "obj"))
  {
    vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New();
    reader->SetFileName(filename.c_str());
    reader->Update();

    if (!reader->GetOutput()->GetNumberOfPoints())
    {
      std::cout << "Failed to load data from: " << filename << std::endl;
    }
    else
    {
      polyDataList.push_back(shallowCopy(reader->GetOutput()));
    }
  }
  else if (endsWith(boost::to_lower_copy(filename), "stl"))
  {
    vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
    reader->SetFileName(filename.c_str());
    reader->Update();

    if (!reader->GetOutput()->GetNumberOfPoints())
    {
      std::cout << "Failed to load data from: " << filename << std::endl;
    }
    else
    {
      polyDataList.push_back(shallowCopy(reader->GetOutput()));
    }
  }
  else if (endsWith(boost::to_lower_copy(filename), "vtp"))
  {
    vtkSmartPointer<vtkXMLPolyDataReader> reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
    reader->SetFileName(filename.c_str());
    reader->Update();

    if (!reader->GetOutput()->GetNumberOfPoints())
    {
      std::cout << "Failed to load data from: " << filename << std::endl;
    }
    else
    {
      polyDataList.push_back(shallowCopy(reader->GetOutput()));
    }
  }
  else if (endsWith(boost::to_lower_copy(filename), "vtm"))
  {
    vtkSmartPointer<vtkXMLMultiBlockDataReader> reader = vtkSmartPointer<vtkXMLMultiBlockDataReader>::New();
    reader->SetFileName(filename.c_str());
    reader->Update();

    vtkMultiBlockDataSet* mb = vtkMultiBlockDataSet::SafeDownCast(reader->GetOutput());
    if (mb)
    {
      for (int i = 0; i < mb->GetNumberOfBlocks(); ++i)
      {
        vtkPolyData* polyData = vtkPolyData::SafeDownCast(mb->GetBlock(i));
        if (polyData && polyData->GetNumberOfPoints())
        {
        polyDataList.push_back(shallowCopy(polyData));
        }
      }
    }
  }

  return polyDataList;
}

/*
void QuaternionToAngleAxis(double wxyz[4], double angleAxis[4])
{
  vtkQuaternion<double> quat(wxyz);
  angleAxis[0] = quat.GetRotationAngleAndAxis(angleAxis+1);
  angleAxis[0] = vtkMath::DegreesFromRadians(angleAxis[0]);
}
*/

namespace {

typedef std::map<std::string, vtkSmartPointer<vtkTexture> > TextureMapType;
TextureMapType TextureMap;

}

vtkSmartPointer<vtkTexture> getTextureForMesh(vtkSmartPointer<vtkPolyData> polyData, const std::string& meshFileName)
{
  vtkStringArray* textureArray = vtkStringArray::SafeDownCast(polyData->GetFieldData()->GetAbstractArray("texture_filename"));
  if (!textureArray)
  {
    return 0;
  }

  std::string textureFileName = textureArray->GetValue(0);
  //if (boost::filesystem::path(textureFileName).is_relative())
  //{
    std::string baseDir = boost::filesystem::path(meshFileName).parent_path().native();
    textureFileName = baseDir + "/" + textureFileName;
  //}

  if (!boost::filesystem::exists(textureFileName))
  {
    printf("cannot find texture file: %s\n", textureFileName.c_str());
    return 0;
  }



  TextureMapType::const_iterator itr = TextureMap.find(textureFileName);
  if (itr != TextureMap.end())
  {
    return itr->second;
  }


  vtkSmartPointer<vtkImageData> image = loadImage(textureFileName);
  if (!image)
  {
    return 0;
  }

  vtkSmartPointer<vtkTexture> texture = vtkSmartPointer<vtkTexture>::New();
  texture->SetInput(image);
  texture->EdgeClampOn();
  texture->RepeatOn();
  TextureMap[textureFileName] = texture;

  return texture;
}

ddMeshVisual::Ptr visualFromPolyData(vtkSmartPointer<vtkPolyData> polyData)
{
  ddMeshVisual::Ptr visual(new ddMeshVisual);
  visual->PolyData = computeNormals(polyData);
  visual->Actor = vtkSmartPointer<vtkActor>::New();
  visual->Transform = vtkSmartPointer<vtkTransform>::New();
  visual->Actor->SetUserTransform(visual->Transform);

  visual->Actor->GetProperty()->SetSpecular(0.9);
  visual->Actor->GetProperty()->SetSpecularPower(20);
  visual->Actor->GetProperty()->SetColor(GRAY_DEFAULT/255.0, GRAY_DEFAULT/255.0, GRAY_DEFAULT/255.0);
  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  SetInputData(mapper, visual->PolyData);
  visual->Actor->SetMapper(mapper);

  bool useShadows = false;
  if (useShadows)
  {
    vtkSmartPointer<vtkTransform> shadowT = vtkSmartPointer<vtkTransform>::New();
    vtkSmartPointer<vtkMatrix4x4> mat = vtkSmartPointer<vtkMatrix4x4>::New();
    mat->DeepCopy(shadowT->GetMatrix());
    mat->SetElement(0, 2, -1.0);
    mat->SetElement(1, 2, -1.0);
    mat->SetElement(2, 2, 0.0);
    shadowT->SetMatrix(mat);
    shadowT->PreMultiply();
    shadowT->Concatenate(visual->Transform);

    visual->ShadowActor = vtkSmartPointer<vtkActor>::New();
    visual->ShadowActor->SetMapper(mapper);
    visual->ShadowActor->SetUserTransform(shadowT);
    visual->ShadowActor->GetProperty()->LightingOff();
    visual->ShadowActor->GetProperty()->SetColor(0, 0, 0);
  }

  return visual;
}

std::vector<ddMeshVisual::Ptr> loadMeshVisuals(const std::string& filename)
{
  std::vector<ddMeshVisual::Ptr> visuals;

  std::vector<vtkSmartPointer<vtkPolyData> > polyDataList = loadPolyData(filename);

  for (size_t i = 0; i < polyDataList.size(); ++i)
  {
    ddMeshVisual::Ptr visual = visualFromPolyData(polyDataList[i]);
    if (!visual)
    {
      continue;
    }

    visual->Texture = getTextureForMesh(polyDataList[i], filename);
    //visual->Actor->SetTexture(visual->Texture);
    visuals.push_back(visual);
  }

  return visuals;
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

class URDFRigidBodyManipulatorVTK : public RigidBodyManipulator
{
public:

  typedef std::map<std::shared_ptr<RigidBody>, std::vector<ddMeshVisual::Ptr> > MeshMapType;

  MeshMapType meshMap;

  std::map<std::string, int> dofMap;

  std::unordered_set<std::string> fixedDOFs;

  ddPtrMacro(URDFRigidBodyManipulatorVTK);

  URDFRigidBodyManipulatorVTK() : RigidBodyManipulator()
  {}

  void computeDofMap()
  {
    this->dofMap.clear();

    RigidBodyManipulator* model = this;

    const std::shared_ptr<RigidBody> worldBody = model->bodies[0];

    for (size_t bodyIndex = 0; bodyIndex < model->bodies.size(); ++bodyIndex)
    {

      std::shared_ptr<RigidBody> body = model->bodies[bodyIndex];

      if (!body->hasParent())
      {
        continue;
      }
      
      if (body->getJoint().getNumPositions() == 0)
      {
        fixedDOFs.insert(body->getJoint().getName());
        continue;
      }

      int dofId = body->position_num_start;

      if (body->parent == worldBody)
      {
        //printf("dofMap base\n");

        dofMap["base_x"] = dofId + 0;
        dofMap["base_y"] = dofId + 1;
        dofMap["base_z"] = dofId + 2;
        dofMap["base_roll"] = dofId + 3;
        dofMap["base_pitch"] = dofId + 4;
        dofMap["base_yaw"] = dofId + 5;
      }
      else
      {
        //printf("dofMap[%s] = %d\n", body->getJoint().getName().c_str(), dofId);
        dofMap[body->getJoint().getName()] = dofId;
      }

    }
  }

  virtual ~URDFRigidBodyManipulatorVTK()
  {

  }

  std::vector<ddMeshVisual::Ptr> meshVisuals()
  {
    std::vector<ddMeshVisual::Ptr> visuals;

    for (MeshMapType::iterator itr = meshMap.begin(); itr != meshMap.end(); ++itr)
    {
      for (size_t i = 0; i < itr->second.size(); ++i)
      {
        visuals.push_back(itr->second[i]);
      }
    }

    return visuals;
  }


  std::string locateMeshFile(const std::string& meshFilename, const std::string& root_dir)
  {

    string fname = meshFilename;
    bool has_package = boost::find_first(meshFilename,"package://");

    if (has_package)
    {
      //cout << "replacing " << fname;
      boost::replace_first(fname,"package://","");
      string package = fname.substr(0,fname.find_first_of("/"));

      QString packageDir = ddDrakeModel::findPackageDirectory(package.c_str());
      if (packageDir.isEmpty())
      {
        std::cout << "Failed to locate package: " << package << " in filename: " << fname << std::endl;
        return std::string();
      }


      //boost::replace_first(fname,package,rospack(package));

      boost::replace_first(fname, package, packageDir.toAscii().data());
      //cout << " with " << fname << endl;
    }
    else
    {
      fname = root_dir + "/" + meshFilename;
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
    //std::string fileExtension = boost::to_lower_copy(mypath.extension().native());

    std::vector<std::string> supportedExtensions;
    supportedExtensions.push_back(".vtm");
    supportedExtensions.push_back(".vtp");
    supportedExtensions.push_back(".obj");
    supportedExtensions.push_back(".stl");

    for (size_t i = 0; i < supportedExtensions.size(); ++i)
    {
      std::string fileWithExtension = mypath.replace_extension(supportedExtensions[i]).native();

      if (boost::filesystem::exists(fileWithExtension))
      {
        return fileWithExtension;
      }

      fileWithExtension = mypath.replace_extension(boost::to_upper_copy(supportedExtensions[i])).native();
      if (boost::filesystem::exists(fileWithExtension))
      {
        return fileWithExtension;
      }

    }

    cerr << "Warning: Mesh " << fname << " ignored because it does not have supported file extension (obj, stl)" << endl;
    return std::string();
  }


  void loadVisuals(const std::string& root_dir=".")
  {

    //printf("load visuals...\n");
    RigidBodyManipulator* model = this;

    for (size_t bodyIndex = 0; bodyIndex < model->bodies.size(); ++bodyIndex)
    {

      std::shared_ptr<RigidBody> body = model->bodies[bodyIndex];

      //printf("body: %s\n", body->linkname.c_str());

      for (size_t visualIndex = 0 ; visualIndex < body->visual_elements.size(); ++visualIndex)
      {
        //printf("vi %d\n", visualIndex);

        const DrakeShapes::VisualElement& visual = body->visual_elements[visualIndex];

        const DrakeShapes::Shape visualType = visual.getShape();

        //printf("shape: %d\n", visualType);

        std::vector<ddMeshVisual::Ptr> loadedVisuals;


        if (visualType == DrakeShapes::MESH)
        {

          const DrakeShapes::Mesh& mesh = static_cast<const DrakeShapes::Mesh&>(visual.getGeometry());

          std::string filename = locateMeshFile(mesh.filename, root_dir);
          if (filename.size())
          {
            //printf("loading mesh: %s\n", filename.c_str());
            loadedVisuals = loadMeshVisuals(filename);
          }

        }
        else if (visualType == DrakeShapes::SPHERE)
        {
          const DrakeShapes::Sphere& sphere = static_cast<const DrakeShapes::Sphere&>(visual.getGeometry());
          double radius = sphere.radius;
          loadedVisuals.push_back(makeSphereVisual(radius));
        }
        else if (visualType == DrakeShapes::BOX)
        {
          const DrakeShapes::Box& box = static_cast<const DrakeShapes::Box&>(visual.getGeometry());

          loadedVisuals.push_back(makeBoxVisual(box.size[0], box.size[1], box.size[2]));
        }
        else if (visualType == DrakeShapes::CYLINDER)
        {
          const DrakeShapes::Cylinder& cyl = static_cast<const DrakeShapes::Cylinder&>(visual.getGeometry());
          loadedVisuals.push_back(makeCylinderVisual(cyl.radius, cyl.length));
        }
        else if (visualType == DrakeShapes::CAPSULE)
        {
          const DrakeShapes::Capsule& cyl = static_cast<const DrakeShapes::Capsule&>(visual.getGeometry());
          loadedVisuals.push_back(makeCylinderVisual(cyl.radius, cyl.length));
        }

        for (size_t mvi = 0; mvi < loadedVisuals.size(); ++mvi)
        {
          ddMeshVisual::Ptr meshVisual = loadedVisuals[mvi];

          meshVisual->VisualToLink = makeTransform(visual.getLocalTransform());

          meshVisual->Name = body->linkname;
          meshMap[body].push_back(meshVisual);

          meshVisual->Actor->GetProperty()->SetColor(visual.getMaterial()[0],
                                                     visual.getMaterial()[1],
                                                     visual.getMaterial()[2]);
        }

      }
    }

    //printf("done\n");

  }


  virtual void updateModel()
  {
    const Vector4d zero(0,0,0,1);
    double theta, axis[3], quat[4];
    double angleAxis[4];
    Matrix<double,7,1> pose;

    RigidBodyManipulator* model = this;



    for (size_t bodyIndex = 0; bodyIndex < model->bodies.size(); ++bodyIndex)
    {

      std::shared_ptr<RigidBody> body = model->bodies[bodyIndex];

      MeshMapType::iterator itr = meshMap.find(body);
      if (itr == this->meshMap.end())
      {
        continue;
      }

      //model->forwardKin(body->body_index, zero, 2, pose);
      //auto pt = model->forwardKinNew(Vector3d::Zero().eval(), body->body_index, 0, 2, 0);
      //pose = pt.value();

      pose = model->forwardKinNew(Vector3d::Zero().eval(), body->body_index, 0, 2, 0).value();

      double* posedata = pose.data();

      bot_quat_to_angle_axis(&posedata[3], &angleAxis[0], &angleAxis[1]);
      angleAxis[0] = vtkMath::DegreesFromRadians(angleAxis[0]);

      vtkSmartPointer<vtkTransform> linkToWorld = vtkSmartPointer<vtkTransform>::New();
      linkToWorld->PostMultiply();
      linkToWorld->RotateWXYZ(angleAxis[0], angleAxis[1], angleAxis[2], angleAxis[3]);
      linkToWorld->Translate(pose(0), pose(1), pose(2));

      //printf("%s to world: %f %f %f\n", body->linkname.c_str(), pose(0), pose(1), pose(2));

      for (size_t visualIndex = 0; visualIndex < itr->second.size(); ++visualIndex)
      {
        ddMeshVisual::Ptr meshVisual = itr->second[visualIndex];


        vtkSmartPointer<vtkTransform> visualToWorld = vtkSmartPointer<vtkTransform>::New();
        visualToWorld->PostMultiply();
        visualToWorld->Concatenate(meshVisual->VisualToLink);
        visualToWorld->Concatenate(linkToWorld);
        meshVisual->Transform->SetMatrix(visualToWorld->GetMatrix());

      }

    }

  }


  virtual QMap<QString, int> getLinkIds()
  {
    QMap<QString, int> linkMap;


    RigidBodyManipulator* model = this;

    for (size_t bodyIndex = 0; bodyIndex < model->bodies.size(); ++bodyIndex)
    {
      std::shared_ptr<RigidBody> body = model->bodies[bodyIndex];

      if (body->linkname.size())
      {
        linkMap[body->linkname.c_str()] = body->body_index;
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

    RigidBodyManipulator* model = this;

    pose = model->forwardKinNew(Vector3d::Zero().eval(), linkMap.value(linkName), 0, 2, 0).value();

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

    if (!model->addRobotFromURDFString(xml_string, PackageSearchPaths, pathname))
    {
      return URDFRigidBodyManipulatorVTK::Ptr();
    }

  }

  model->computeDofMap();
  model->loadVisuals();
  return model;
}


URDFRigidBodyManipulatorVTK::Ptr loadVTKModelFromXML(const string &xmlString)
{
  URDFRigidBodyManipulatorVTK::Ptr model(new URDFRigidBodyManipulatorVTK);
  if (!model->addRobotFromURDFString(xmlString,PackageSearchPaths, ""))
  {
    return URDFRigidBodyManipulatorVTK::Ptr();
  }

  model->computeDofMap();
  model->loadVisuals();
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
    this->TexturesEnabled = false;
    this->Alpha = 1.0;
    this->Color = QColor(GRAY_DEFAULT, GRAY_DEFAULT, GRAY_DEFAULT);
  }

  URDFRigidBodyManipulatorVTK::Ptr Model;

  QString FileName;
  bool Visible;
  bool TexturesEnabled;
  double Alpha;
  QColor Color;
  QVector<double> JointPositions;
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
const ddSharedPtr<RigidBodyManipulator> ddDrakeModel::getDrakeRBM() const
{
  return this->Internal->Model;
}

//-----------------------------------------------------------------------------
int ddDrakeModel::numberOfJoints()
{
  if (!this->Internal->Model)
  {
    return 0;
  }

  return this->Internal->Model->num_positions;
}

//-----------------------------------------------------------------------------
void ddDrakeModel::setJointPositions(const QVector<double>& jointPositions, const QList<QString>& jointNames)
{
  URDFRigidBodyManipulatorVTK::Ptr model = this->Internal->Model;

  if (!model)
  {
    std::cout << "ddDrakeModel::setJointPositions(): model is null" << std::endl;
    return;
  }


  if (jointPositions.size() != jointNames.size())
  {
    std::cout << "ddDrakeModel::setJointPositions(): input jointPositions size "
              << jointPositions.size() << " != " << jointNames.size() << std::endl;
    return;
  }

  if (this->Internal->JointPositions.size() != model->num_positions)
  {
    std::cout << "Internal joint positions vector has inconsistent size." << std::endl;
    return;
  }

  for (int i = 0; i < jointNames.size(); ++i)
  {
    const QString& dofName = jointNames[i];

    std::map<std::string, int>::const_iterator itr = model->dofMap.find(dofName.toAscii().data());
    if (itr == model->dofMap.end())
    {
      std::unordered_set<std::string>::const_iterator itr_fixed = model->fixedDOFs.find(dofName.toAscii().data());
      if (itr_fixed == model->fixedDOFs.end())
      {
        printf("Could not find URDF model dof with name: %s\n", qPrintable(dofName));
      }
      continue;
    }

    int dofId = itr->second;
    this->Internal->JointPositions[dofId] = jointPositions[i];

    //printf("setJoint %s --> %d, %f\n", qPrintable(dofName), dofId, jointPositions[i]);
  }

  this->setJointPositions(this->Internal->JointPositions);
}

//-----------------------------------------------------------------------------
void ddDrakeModel::setJointPositions(const QVector<double>& jointPositions)
{
  URDFRigidBodyManipulatorVTK::Ptr model = this->Internal->Model;

  if (!model)
  {
    std::cout << "ddDrakeModel::setJointPositions(): model is null" << std::endl;
    return;
  }

  if (jointPositions.size() != model->num_positions)
  {
    std::cout << "ddDrakeModel::setJointPositions(): input jointPositions size "
              << jointPositions.size() << " != " << model->num_positions << std::endl;
    return;
  }

  VectorXd q = VectorXd::Zero(model->num_positions);
  VectorXd v = VectorXd::Zero(model->num_velocities);
  for (int i = 0; i < jointPositions.size(); ++i)
  {
    q(i) = jointPositions[i];
  }

  this->Internal->JointPositions = jointPositions;
  model->doKinematicsNew(q, v, false, false);
  model->updateModel();
  emit this->modelChanged();
}

//-----------------------------------------------------------------------------
const QVector<double>& ddDrakeModel::getJointPositions() const
{
  return this->Internal->JointPositions;
}

//-----------------------------------------------------------------------------
QVector<double> ddDrakeModel::getJointPositions(const QList<QString>& jointNames) const
{
  URDFRigidBodyManipulatorVTK::Ptr model = this->Internal->Model;
  QVector<double> ret(jointNames.size());

  if (!model)
  {
    std::cout << "ddDrakeModel::getJointPositions(): model is null" << std::endl;
    return ret;
  }

  if (this->Internal->JointPositions.size() != model->num_positions)
  {
    std::cout << "Internal joint positions vector has inconsistent size." << std::endl;
    return ret;
  }

  for (int i = 0; i < jointNames.size(); ++i)
  {
    const QString& dofName = jointNames[i];

    std::map<std::string, int>::const_iterator itr = model->dofMap.find(dofName.toAscii().data());
    if (itr == model->dofMap.end())
    {
      std::unordered_set<std::string>::const_iterator itr_fixed = model->fixedDOFs.find(dofName.toAscii().data());
      if (itr_fixed == model->fixedDOFs.end())
      {
        printf("Could not find URDF model dof with name: %s\n", qPrintable(dofName));
      }
      continue;
    }
    int dofId = itr->second;
    ret[i] = this->Internal->JointPositions[dofId];

    //printf("setJoint %s --> %d, %f\n", qPrintable(dofName), dofId, jointPositions[i]);
  }
  return ret;
}

//-----------------------------------------------------------------------------
QVector<double> ddDrakeModel::getCenterOfMass() const
{
  URDFRigidBodyManipulatorVTK::Ptr model = this->Internal->Model;
  Vector3d com;
  model->getCOM(com);
  QVector<double> ret;
  ret << com[0] << com[1] << com[2];
  return ret;
}

//-----------------------------------------------------------------------------
QVector<double> ddDrakeModel::getBodyContactPoints(const QString& bodyName) const
{
  QVector<double> ret;
  URDFRigidBodyManipulatorVTK::Ptr model = this->Internal->Model;

  for (size_t bodyIndex = 0; bodyIndex < model->bodies.size(); ++bodyIndex)
  {
    std::shared_ptr<RigidBody> body = model->bodies[bodyIndex];
    if (body->linkname.c_str() == bodyName)
    {
      for (size_t i = 0; i < body->contact_pts.cols(); ++i)
      {
        ret << body->contact_pts(0,i) << body->contact_pts(1,i) << body->contact_pts(2,1);
      }
    }
  }
  return ret;
}

//-----------------------------------------------------------------------------
QVector<double> ddDrakeModel::getJointLimits(const QString& jointName) const
{
  QVector<double> limits;
  limits << 0.0 << 0.0;

  URDFRigidBodyManipulatorVTK::Ptr model = this->Internal->Model;

  if (!model)
  {
    std::cout << "ddDrakeModel::getJointLimits(): model is null" << std::endl;
    return limits;
  }

  std::map<std::string, int>::const_iterator itr = model->dofMap.find(jointName.toAscii().data());
  if (itr == model->dofMap.end())
  {
    printf("Could not find URDF model dof with name: %s\n", qPrintable(jointName));
    return limits;
  }

  int dofId = itr->second;


  limits[0] = this->Internal->Model->joint_limit_min[dofId];
  limits[1] = this->Internal->Model->joint_limit_max[dofId];
  return limits;
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
int ddDrakeModel::findLinkID(const QString& linkName) const
{
  return this->Internal->Model->findLinkId(linkName.toAscii().data(), -1);
}

//-----------------------------------------------------------------------------
QList<QString> ddDrakeModel::getJointNames()
{
  if (!this->Internal->Model)
  {
    return QList<QString>();
  }

  // For convenience when setting joint values directly,
  // return this list of names in the same order as the list
  // of joint values will come out.
  QList<QString> names;
  names.reserve(this->Internal->Model->dofMap.size());
  for (int i=0; i < this->Internal->Model->dofMap.size(); i++)
    names.append(QString());

  std::map<std::string, int>::const_iterator itr;
  for(itr = this->Internal->Model->dofMap.begin(); itr != this->Internal->Model->dofMap.end(); ++itr)
  {
    names[itr->second] = QString(itr->first.c_str());
  }

  return names;
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
bool ddDrakeModel::loadFromFile(const QString& filename)
{
  URDFRigidBodyManipulatorVTK::Ptr model = loadVTKModelFromFile(filename.toAscii().data());
  if (!model)
  {
    return false;
  }

  this->Internal->FileName = filename;
  this->Internal->Model = model;

  this->setJointPositions(QVector<double>(model->num_positions, 0.0));
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

  this->setJointPositions(QVector<double>(model->num_positions, 0.0));
  return true;
}

//-----------------------------------------------------------------------------
void ddDrakeModel::getModelMesh(vtkPolyData* polyData)
{
  if (!polyData)
  {
    return;
  }

  std::vector<ddMeshVisual::Ptr> visuals = this->Internal->Model->meshVisuals();
  vtkSmartPointer<vtkAppendPolyData> appendFilter = vtkSmartPointer<vtkAppendPolyData>::New();

  for (size_t i = 0; i < visuals.size(); ++i)
  {
    AddInputData(appendFilter, transformPolyData(visuals[i]->PolyData, visuals[i]->Transform));
  }

  if (visuals.size())
  {
    appendFilter->Update();
  }

  polyData->DeepCopy(appendFilter->GetOutput());
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
    if (visuals[i]->ShadowActor)
    {
      renderer->AddActor(visuals[i]->ShadowActor);
    }
  }
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
    if (visuals[i]->ShadowActor)
    {
      renderer->RemoveActor(visuals[i]->ShadowActor);
    }
  }
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
  emit this->displayChanged();
}

//-----------------------------------------------------------------------------
bool ddDrakeModel::texturesEnabled() const
{
  return this->Internal->TexturesEnabled;
}

//-----------------------------------------------------------------------------
void ddDrakeModel::setTexturesEnabled(bool enabled)
{
  this->Internal->TexturesEnabled = enabled;


  std::vector<ddMeshVisual::Ptr> visuals = this->Internal->Model->meshVisuals();
  for (size_t i = 0; i < visuals.size(); ++i)
  {
    visuals[i]->Actor->SetTexture(enabled ? visuals[i]->Texture : NULL);
  }

  emit this->displayChanged();
}

//-----------------------------------------------------------------------------
QColor ddDrakeModel::getLinkColor(const QString& linkName) const
{
  std::vector<ddMeshVisual::Ptr> visuals = this->Internal->Model->meshVisuals();
  for (size_t i = 0; i < visuals.size(); ++i)
  {
    if (visuals[i]->Name == linkName.toAscii().data())
    {
      double* rgb = visuals[i]->Actor->GetProperty()->GetColor();
      double alpha = visuals[i]->Actor->GetProperty()->GetOpacity();
      return QColor(rgb[0]*255, rgb[1]*255, rgb[2]*255, alpha*255);
    }
  }
  return QColor();
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

      if (visuals[i]->ShadowActor)
      {
        visuals[i]->ShadowActor->GetProperty()->SetOpacity(alpha);
      }
    }
  }
  emit this->displayChanged();
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
    if (visuals[i]->ShadowActor)
    {
      visuals[i]->ShadowActor->GetProperty()->SetOpacity(alpha);
    }
  }
  this->Internal->Alpha = alpha;
  emit this->displayChanged();
}

//-----------------------------------------------------------------------------
void ddDrakeModel::setVisible(bool visible)
{
  std::vector<ddMeshVisual::Ptr> visuals = this->Internal->Model->meshVisuals();

  for (size_t i = 0; i < visuals.size(); ++i)
  {
    visuals[i]->Actor->SetVisibility(visible);
    if (visuals[i]->ShadowActor)
    {
      visuals[i]->ShadowActor->SetVisibility(visible);
    }
  }
  this->Internal->Visible = visible;
  emit this->displayChanged();
}

//-----------------------------------------------------------------------------
bool ddDrakeModel::visible() const
{
  return this->Internal->Visible;
}

//-----------------------------------------------------------------------------
void ddDrakeModel::addPackageSearchPath(const QString& searchPath)
{
  std::string packageName = QDir(searchPath).dirName().toAscii().data();
  if (PackageSearchPaths.count(packageName) == 0)
  {
    PackageSearchPaths[packageName] = searchPath.toAscii().data();
  }
}

//-----------------------------------------------------------------------------
QString ddDrakeModel::findPackageDirectory(const QString& packageName)
{
  auto packageDirIter = PackageSearchPaths.find(packageName.toAscii().data());
  if (packageDirIter != PackageSearchPaths.end()) {
    return QString::fromStdString(packageDirIter->second);
  } else {
    return QString();
  }
}
