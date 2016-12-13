#include "ddDrakeModel.h"
#include "ddSharedPtr.h"

#include <drake/multibody/joints/floating_base_types.h>
#include <drake/multibody/parsers/package_map.h>
#include <drake/multibody/parsers/urdf_parser.h>
#include <drake/multibody/rigid_body_tree.h>
#include <drake/multibody/shapes/geometry.h>

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
#include <math.h>

#include <QFileInfo>
#include <QTextStream>
#include <QMap>
#include <QDir>

using std::map;
using std::vector;
using std::string;
using std::istringstream;
using namespace Eigen;

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
  QColor Color;
  std::string Name;

private:

  Q_DISABLE_COPY(ddMeshVisual);
};

namespace
{

const int GRAY_DEFAULT = 190;

drake::parsers::PackageMap PackageSearchPaths;

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

vtkSmartPointer<vtkTransform> makeTransform( const Eigen::Isometry3d& mat)
{
  vtkSmartPointer<vtkMatrix4x4> vtkmat = vtkSmartPointer<vtkMatrix4x4>::New();
  for (int i = 0; i < 4; ++i)
    {
    for (int j = 0; j < 4; ++j)
      {
      vtkmat->SetElement(i, j, mat.matrix()(i,j));
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

vtkSmartPointer<vtkPolyData> scalePolyData(vtkPolyData* polyData,
                                           const Eigen::Vector3d& scale)
{
  vtkSmartPointer<vtkTransform> t = vtkSmartPointer<vtkTransform>::New();
  t->Scale(scale(0), scale(1), scale(2));
  return transformPolyData(polyData, t);
}

vtkSmartPointer<vtkImageData> loadImage(const QString& filename)
{
  vtkSmartPointer<vtkImageData> image;
  QString ext = QFileInfo(filename).suffix().toLower();

  if (ext == "jpg")
  {
    vtkSmartPointer<vtkJPEGReader> reader = vtkSmartPointer<vtkJPEGReader>::New();
    reader->SetFileName(filename.toLatin1().constData());
    reader->Update();
    image = reader->GetOutput();
  }
  else if (ext == "png")
  {
    vtkSmartPointer<vtkPNGReader> reader = vtkSmartPointer<vtkPNGReader>::New();
    reader->SetFileName(filename.toLatin1().constData());
    reader->Update();
    image = reader->GetOutput();
  }

  if (!image->GetNumberOfPoints())
  {
    std::cout << "Failed to load data from: " << qPrintable(filename) << std::endl;
    return 0;
  }

  return image;

}

std::vector<vtkSmartPointer<vtkPolyData> > loadPolyData(const QString& filename)
{
  std::vector<vtkSmartPointer<vtkPolyData> > polyDataList;
  QString ext = QFileInfo(filename).suffix().toLower();

  if (ext == "obj")
  {
    vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New();
    reader->SetFileName(filename.toLatin1().constData());
    reader->Update();

    if (!reader->GetOutput()->GetNumberOfPoints())
    {
      std::cout << "Failed to load data from: " << qPrintable(filename) << std::endl;
    }
    else
    {
      polyDataList.push_back(shallowCopy(reader->GetOutput()));
    }
  }
  else if (ext == "stl")
  {
    vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
    reader->SetFileName(filename.toLatin1().constData());
    reader->Update();

    if (!reader->GetOutput()->GetNumberOfPoints())
    {
      std::cout << "Failed to load data from: " << qPrintable(filename) << std::endl;
    }
    else
    {
      polyDataList.push_back(shallowCopy(reader->GetOutput()));
    }
  }
  else if (ext == "vtp")
  {
    vtkSmartPointer<vtkXMLPolyDataReader> reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
    reader->SetFileName(filename.toLatin1().constData());
    reader->Update();

    if (!reader->GetOutput()->GetNumberOfPoints())
    {
      std::cout << "Failed to load data from: " << qPrintable(filename) << std::endl;
    }
    else
    {
      polyDataList.push_back(shallowCopy(reader->GetOutput()));
    }
  }
  else if (ext == "vtm")
  {
    vtkSmartPointer<vtkXMLMultiBlockDataReader> reader = vtkSmartPointer<vtkXMLMultiBlockDataReader>::New();
    reader->SetFileName(filename.toLatin1().constData());
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


namespace {

typedef QMap<QString, vtkSmartPointer<vtkTexture> > TextureMapType;
TextureMapType TextureMap;

}

vtkSmartPointer<vtkTexture> getTextureForMesh(vtkSmartPointer<vtkPolyData> polyData, const QString& meshFileName)
{
  vtkStringArray* textureArray = vtkStringArray::SafeDownCast(polyData->GetFieldData()->GetAbstractArray("texture_filename"));
  if (!textureArray)
  {
    return 0;
  }

  QString textureFileName = textureArray->GetValue(0).c_str();
  if (QFileInfo(textureFileName).isRelative())
  {
    QString baseDir = QFileInfo(meshFileName).absolutePath();
    textureFileName = baseDir + "/" + textureFileName;
  }

  if (!QFileInfo(textureFileName).exists())
  {
    printf("cannot find texture file: %s\n", qPrintable(textureFileName));
    return 0;
  }


  if (TextureMap.contains(textureFileName))
  {
    return TextureMap[textureFileName];
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

std::vector<ddMeshVisual::Ptr> loadMeshVisuals(const QString& filename)
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

class URDFRigidBodyTreeVTK : public RigidBodyTreed
{
public:

  typedef std::map<const RigidBody<double>*, std::vector<ddMeshVisual::Ptr> > MeshMapType;

  std::shared_ptr<KinematicsCache<double> > cache;

  MeshMapType meshMap;

  std::map<std::string, int> dofMap;

  std::unordered_set<std::string> fixedDOFs;

  ddPtrMacro(URDFRigidBodyTreeVTK);

  URDFRigidBodyTreeVTK() : RigidBodyTree()
  {}

  void computeDofMap()
  {
    this->dofMap.clear();

    RigidBodyTreed* model = this;

    const RigidBody<double>& worldBody = model->world();
    for (const RigidBody<double>* body : model->FindModelInstanceBodies(
             worldBody.get_model_instance_id())) {
      if (!body->has_parent_body())
      {
        continue;
      }

      if (body->getJoint().get_num_positions() == 0)
      {
        fixedDOFs.insert(body->getJoint().get_name());
        continue;
      }

      int dofId = body->get_position_start_index();

      if (body->has_as_parent(worldBody))
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
        //printf("dofMap[%s] = %d\n", body->getJoint().get_name().c_str(), dofId);
        dofMap[body->getJoint().get_name()] = dofId;
      }

    }
  }

  virtual ~URDFRigidBodyTreeVTK()
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

  void replaceFirst(QString& inputStr, const QString& subStr, const QString& replacement)
  {
    if (inputStr.indexOf(subStr) >= 0)
      inputStr.replace(inputStr.indexOf(subStr), subStr.size(), replacement);
  }

  QString replaceExtension(const QString& inputStr, const QString& newExtension)
  {
    return inputStr.left(inputStr.size() - QFileInfo(inputStr).suffix().size()) + newExtension;
  }

  QString locateMeshFile(const QString& meshFilename, const QString& rootDir)
  {
    bool hasPackage = meshFilename.startsWith("package://");
    QString fname = meshFilename;

    if (hasPackage)
    {

      // if there is a package:// prefix then we
      // are looking at a string of the form: package://<package_name>/path/to/mesh.obj

      replaceFirst(fname, "package://", "");

      QString package = fname.left(fname.indexOf("/"));
      QString packageDir = ddDrakeModel::findPackageDirectory(package);
      if (packageDir.isEmpty())
      {
        std::cout << "Failed to locate package: " << qPrintable(package) << " in filename: " << qPrintable(fname) << std::endl;
        return QString();
      }

      replaceFirst(fname, package, packageDir.toLatin1().data());
    }
    else if (QFileInfo(meshFilename).isRelative())
    {
      fname = rootDir + "/" + meshFilename;
    }

    fname = QFileInfo(fname).absoluteFilePath();

    if (!QFileInfo(fname).exists())
    {
      cerr << "cannot find mesh file: " << qPrintable(fname);
      if (hasPackage)
        cerr << " (note: original mesh string had a package://, perhaps a package path issue)";
      cerr << endl;
      return QString();
    }


    std::vector<QString> supportedExtensions;
    supportedExtensions.push_back("vtm");
    supportedExtensions.push_back("vtp");
    supportedExtensions.push_back("obj");
    supportedExtensions.push_back("stl");

    for (size_t i = 0; i < supportedExtensions.size(); ++i)
    {
      QString fileWithExtension = replaceExtension(fname, supportedExtensions[i]);

      if (QFileInfo(fileWithExtension).exists())
      {
        return fileWithExtension;
      }

      fileWithExtension = replaceExtension(fname, supportedExtensions[i].toUpper());

      if (QFileInfo(fileWithExtension).exists())
      {
        return fileWithExtension;
      }

    }

    cerr << "Warning: Mesh " << qPrintable(fname) << " ignored because it could not be located" << endl;
    return QString();
  }


  void loadVisuals(const QString& rootDir=".")
  {

    RigidBodyTreed* model = this;

    for (const RigidBody<double>* body : model->FindModelInstanceBodies(
             model->world().get_model_instance_id())) {

      const auto& visual_elements = body->get_visual_elements();
      for (size_t visualIndex = 0 ; visualIndex < visual_elements.size(); ++visualIndex)
      {

        const DrakeShapes::VisualElement& visual = visual_elements[visualIndex];
        const DrakeShapes::Shape visualType = visual.getShape();
        std::vector<ddMeshVisual::Ptr> loadedVisuals;

        if (visualType == DrakeShapes::MESH)
        {

          const DrakeShapes::Mesh& mesh = static_cast<const DrakeShapes::Mesh&>(visual.getGeometry());

          QString filename = locateMeshFile(mesh.resolved_filename_.c_str(), rootDir);
          if (filename.size())
          {
            loadedVisuals = loadMeshVisuals(filename);
          }

          if (mesh.scale_ != Eigen::Vector3d::Constant(1.0))
          {

            for (size_t mvi = 0; mvi < loadedVisuals.size(); ++mvi)
            {
              ddMeshVisual::Ptr meshVisual = loadedVisuals[mvi];
              meshVisual->PolyData->ShallowCopy(scalePolyData(meshVisual->PolyData, mesh.scale_));
            }
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

          meshVisual->Name = body->get_name();
          meshMap[body].push_back(meshVisual);

          meshVisual->Color = QColor(visual.getMaterial()[0]*255, visual.getMaterial()[1]*255, visual.getMaterial()[2]*255);
          meshVisual->Actor->GetProperty()->SetColor(visual.getMaterial()[0],
                                                     visual.getMaterial()[1],
                                                     visual.getMaterial()[2]);
        }

      }
    }
  }


  virtual void updateModel()
  {
    RigidBodyTreed* model = this;

    for (const RigidBody<double>* body : model->FindModelInstanceBodies(
             model->world().get_model_instance_id())) {
      MeshMapType::iterator itr = meshMap.find(body);
      if (itr == this->meshMap.end())
      {
        continue;
      }

      vtkSmartPointer<vtkTransform> linkToWorld = makeTransform(relativeTransform(*cache, 0, body->get_body_index()));

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


    RigidBodyTreed* model = this;

    for (const RigidBody<double>* body : model->FindModelInstanceBodies(
             model->world().get_model_instance_id())) {
      if (body->get_name().size())
      {
        linkMap[body->get_name().c_str()] = body->get_body_index();
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

    vtkSmartPointer<vtkTransform> linkToWorld = makeTransform(relativeTransform(*cache, 0, linkMap.value(linkName)));

    return linkToWorld;
  }

};

URDFRigidBodyTreeVTK::Ptr loadVTKModelFromXML(const QString& xmlString, const QString& rootDir="")
{
  URDFRigidBodyTreeVTK::Ptr model(new URDFRigidBodyTreeVTK);

  drake::parsers::urdf::AddModelInstanceFromUrdfStringSearchingInRosPackages(
      xmlString.toUtf8().constData(), PackageSearchPaths,
      rootDir.toLatin1().constData(), drake::multibody::joints::kRollPitchYaw,
      nullptr /* weld to frame */, model.get());

  model->computeDofMap();
  model->loadVisuals(rootDir);
  return model;
}

URDFRigidBodyTreeVTK::Ptr loadVTKModelFromFile(const QString &urdfFilename)
{
  QFile f(urdfFilename);

  if (!f.open(QFile::ReadOnly | QFile::Text))
  {
    cerr << "Could not open file [" << qPrintable(urdfFilename) << "] for parsing." << endl;
    return URDFRigidBodyTreeVTK::Ptr();
  }

  QTextStream in(&f);
  QString xmlString = in.readAll();
  f.close();

  QString rootDir = QFileInfo(urdfFilename).dir().absolutePath();
  return loadVTKModelFromXML(xmlString, rootDir);
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

  URDFRigidBodyTreeVTK::Ptr Model;

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
const ddSharedPtr<RigidBodyTreed> ddDrakeModel::getDrakeRBM() const
{
  return this->Internal->Model;
}

//-----------------------------------------------------------------------------
const ddSharedPtr<KinematicsCache<double> > ddDrakeModel::getKinematicsCache() const
{
  return this->Internal->Model->cache;
}

//-----------------------------------------------------------------------------
int ddDrakeModel::numberOfJoints()
{
  if (!this->Internal->Model)
  {
    return 0;
  }

  return this->Internal->Model->get_num_positions();
}

//-----------------------------------------------------------------------------
void ddDrakeModel::setJointPositions(const QVector<double>& jointPositions, const QList<QString>& jointNames)
{
  URDFRigidBodyTreeVTK::Ptr model = this->Internal->Model;

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

  if (this->Internal->JointPositions.size() != model->get_num_positions())
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
  URDFRigidBodyTreeVTK::Ptr model = this->Internal->Model;

  if (!model)
  {
    std::cout << "ddDrakeModel::setJointPositions(): model is null" << std::endl;
    return;
  }

  if (jointPositions.size() != model->get_num_positions())
  {
    std::cout << "ddDrakeModel::setJointPositions(): input jointPositions size "
              << jointPositions.size() << " != " << model->get_num_positions() << std::endl;
    return;
  }

  VectorXd q = VectorXd::Zero(model->get_num_positions());
  VectorXd v = VectorXd::Zero(model->get_num_velocities());
  for (int i = 0; i < jointPositions.size(); ++i)
  {
    q(i) = jointPositions[i];
  }

  this->Internal->JointPositions = jointPositions;

  model->cache = std::make_shared<KinematicsCache<double> >(
      model->CreateKinematicsCache());
  model->cache->initialize(q);
  model->doKinematics(*model->cache);
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
  URDFRigidBodyTreeVTK::Ptr model = this->Internal->Model;
  QVector<double> ret(jointNames.size());

  if (!model)
  {
    std::cout << "ddDrakeModel::getJointPositions(): model is null" << std::endl;
    return ret;
  }

  if (this->Internal->JointPositions.size() != model->get_num_positions())
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
  URDFRigidBodyTreeVTK::Ptr model = this->Internal->Model;
  Vector3d com;
  com = model->centerOfMass<double>(*model->cache);

  QVector<double> ret;
  ret << com[0] << com[1] << com[2];
  return ret;
}

//-----------------------------------------------------------------------------
QVector<double> ddDrakeModel::getBodyContactPoints(const QString& bodyName) const
{
  QVector<double> ret;
  URDFRigidBodyTreeVTK::Ptr model = this->Internal->Model;

  for (const RigidBody<double>* body : model->FindModelInstanceBodies(
           model->world().get_model_instance_id())) {
    if (body->get_name().c_str() == bodyName)
    {
      const auto& contact_pts = body->get_contact_points();
      for (size_t i = 0; i < contact_pts.cols(); ++i)
      {
        ret << contact_pts(0,i) << contact_pts(1,i) << contact_pts(2,i);
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

  URDFRigidBodyTreeVTK::Ptr model = this->Internal->Model;

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
  return this->Internal->Model->FindBodyIndex(linkName.toAscii().data(), -1);
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
  URDFRigidBodyTreeVTK::Ptr model = loadVTKModelFromFile(filename.toAscii().data());
  if (!model)
  {
    return false;
  }

  this->Internal->FileName = filename;
  this->Internal->Model = model;

  this->setJointPositions(QVector<double>(model->get_num_positions(), 0.0));
  return true;
}

//-----------------------------------------------------------------------------
bool ddDrakeModel::loadFromXML(const QString& xmlString)
{
  URDFRigidBodyTreeVTK::Ptr model = loadVTKModelFromXML(xmlString.toAscii().data());
  if (!model)
  {
    return false;
  }

  this->Internal->FileName = "<xml string>";
  this->Internal->Model = model;

  this->setJointPositions(QVector<double>(model->get_num_positions(), 0.0));
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
void ddDrakeModel::setUrdfColors()
{
  //std::cout << "Set Urdf Colors" << std::endl;
  std::vector<ddMeshVisual::Ptr> visuals = this->Internal->Model->meshVisuals();
  for (size_t i = 0; i < visuals.size(); ++i)
  {
    visuals[i]->Actor->GetProperty()->SetColor(visuals[i]->Color.redF(),
                                               visuals[i]->Color.greenF(),
                                               visuals[i]->Color.blueF());
  }
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
  if (!PackageSearchPaths.Contains(packageName))
  {
    PackageSearchPaths.Add(packageName, searchPath.toAscii().data());
  }
}

//-----------------------------------------------------------------------------
QString ddDrakeModel::findPackageDirectory(const QString& packageName)
{
  const std::string package_name = packageName.toAscii().data();
  if (PackageSearchPaths.Contains(package_name)) {
    return QString::fromStdString(PackageSearchPaths.GetPath(package_name));
  } else {
    return QString();
  }
}
