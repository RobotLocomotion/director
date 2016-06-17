/*=========================================================================

Program:   Visualization Toolkit

Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
All rights reserved.
See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

This software is distributed WITHOUT ANY WARRANTY; without even
the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkCollections.h"

#include <vtkObjectFactory.h>
#include <vtkSmartPointer.h>
#include <vtkActor.h>

#include <vtkOpenGL.h>

#include <lcmtypes/vs/object_collection_t.hpp>
#include <lcmtypes/vs_object_collection_t.h>
#include <sstream>

#include <QString>
#include <QFileInfo>

using namespace std;
using namespace octomap;

vtkStandardNewMacro(vtkCollections);





class Collection {
public:
  int id;
  string name;
  int type;
  bool show;

  Collection(int id, string name, int type, bool show) : id(id), name(name), type(type), show(show) {}

  virtual ~Collection() {}
  virtual void draw(void *self, int64_t range_start, int64_t range_end) = 0;
  virtual void clear() = 0;
};

typedef map<int, Collection*> collections_t;

/**
 * A configuration block for a single collection
 */
class CollectionConfig
{
public:
  CollectionConfig();
  
  void set(const char* name, const char* value);
  void set(const string & name, const string & value);
  const string & get(const string & name);

  // TODO: helper function to read arrays, double, int, ...

  bool is_configured() {return m_is_configured;}
  bool has_value(const std::string & name);
private:
  bool m_is_configured;
  map<string, string> m_properties;
};

CollectionConfig::CollectionConfig() : m_is_configured(false) {}
void CollectionConfig::set(const char* name, const char* value)
{
  const std::string a(name);
  const std::string b(value);
  set(a,b);
}

void CollectionConfig::set(const std::string & name, const std::string & value)
{
  m_is_configured = true;
  m_properties[name] = value;
}

bool CollectionConfig::has_value(const std::string & name)
{
  map<string, string>::iterator it = m_properties.find(name);
  return it != m_properties.end();
}

const std::string & CollectionConfig::get(const std::string & name)
{
  return m_properties[name];
}
















class vtkCollections::vtkInternal {
public:
  vtkInternal()
    {
      this->msg.nobjects = 0;
    }

  vs_object_collection_t msg;

  collections_t collections;

  //ViewerWidget* m_glwidget;

  std::vector<vtkSmartPointer<vtkActor> > Actors;

};


//----------------------------------------------------------------------------
vtkCollections::vtkCollections()
{
  this->Internal = new vtkInternal;
}

//----------------------------------------------------------------------------
vtkCollections::~vtkCollections()
{
  delete this->Internal;
}

//----------------------------------------------------------------------------



class ObjCollection : public Collection {
public:
  typedef vs_object_collection_t my_vs_collection_t;
  typedef vs_object_t my_vs_t;

  // index can be time stamp (64 bit!)
  typedef map<int64_t, my_vs_t>  elements_t;

  int64_t maxid;

  elements_t elements;

  static int get_size(const my_vs_collection_t *msg) {
    return msg->nobjects;
  }

  static void copy(const my_vs_collection_t *msg, int i, elements_t& dst_map) {
    dst_map[msg->objects[i].id] = msg->objects[i];
  }

  ObjCollection(int id, string name, int type, bool show) : Collection(id, name, type, show) {}
  virtual ~ObjCollection() {}

  virtual void draw(void *_self, int64_t range_start, int64_t range_end) {
    /*
    RendererCollections *self = (RendererCollections*) _self;
    // preparations

    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glEnable(GL_DEPTH_TEST);

    switch(type) {
    case VS_OBJECT_COLLECTION_T_TREE:
      glEnable(GL_RESCALE_NORMAL);
      glShadeModel(GL_SMOOTH);
      glEnable(GL_LIGHTING);
      glEnable(GL_COLOR_MATERIAL);
      glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
      break;
    }
    for (elements_t::iterator it = elements.begin(); it != elements.end(); it++) {
      vs_object_t& obj = it->second;
      // only draw if within range
      if (obj.id>=range_start && obj.id<=range_end) {

        // Set color
        CollectionConfig & config = collectionConfig[id];
        bool isconf = config.is_configured();
        GLfloat color[4];
        if (isconf && config.has_value(std::string("Color"))) {
          std::string sColor = config.get("Color");
          sscanf(sColor.c_str(), "%f,%f,%f,%f", &(color[0]), &(color[1]), &(color[2]), &(color[3]));
        } else {
          color[0] = colors[3*(id%num_colors)];
          color[1] = colors[3*(id%num_colors)+1];
          color[2] = colors[3*(id%num_colors)+2];
          color[3] = colors[3*(id%num_colors)+3];
        }

        // glColor3fv(&colors[3*(id%num_colors)]);
        glColor4fv(color);

        double z = time_elevation(self, obj.id, obj.z, it->first);

        double size = 0.1; // 0.1m is the size of the plotted poses
        size = size*self->param_pose_width;

        // Retrive euler angles and reverse the order to get rpy:
        // Corresponds to a snippet of code from Matt Antone. (AffordanceUpdater.cpp in map server)
        Eigen::Vector3d obj_ypr = Eigen::Matrix3d(Eigen::Quaterniond(obj.qw, obj.qx, obj.qy, obj.qz)).eulerAngles(2,1,0);
        Eigen::Vector3d obj_rpy = Eigen::Vector3d( obj_ypr[2], obj_ypr[1], obj_ypr[0]);

        bool is_last = (maxid == obj.id);
        switch(type) {
        case VS_OBJECT_COLLECTION_T_SQUARE:
          draw_square (self, obj.x, obj.y, z, obj_rpy(2), size);
          break;
        case VS_OBJECT_COLLECTION_T_POSE:
          draw_triangle (self, obj.x, obj.y, z, obj_rpy(2), size, is_last);
          break;
        case VS_OBJECT_COLLECTION_T_POSE3D:
          draw_tetra (self, obj.x, obj.y, z, obj_rpy(2), obj_rpy(1), obj_rpy(0), size, is_last);
          break;
        case VS_OBJECT_COLLECTION_T_AXIS3D:
          draw_axis (self, obj.x, obj.y, z, obj_rpy(2), obj_rpy(1), obj_rpy(0), size, is_last);
          break;
        case VS_OBJECT_COLLECTION_T_TREE:
          draw_tree (self, obj.x, obj.y, z);
          break;
        case VS_OBJECT_COLLECTION_T_TAG:
          draw_tag (self, obj.x, obj.y, z, obj_rpy(2), obj_rpy(1), obj_rpy(0));
          break;
        case VS_OBJECT_COLLECTION_T_CAMERA:
          draw_camera (self, obj.x, obj.y, z, obj_rpy(2), obj_rpy(1), obj_rpy(0), size, is_last);
          draw_axis(self, obj.x, obj.y, obj.z, obj_rpy(2), obj_rpy(1), obj_rpy(0), size, is_last);
          break;
        case VS_OBJECT_COLLECTION_T_TRIANGLE:
          draw_equilateral_triangle (self, obj.x, obj.y, z, obj_rpy(2), size, is_last );
          break;
        case VS_OBJECT_COLLECTION_T_HEXAGON:
          draw_hexagon (self, obj.x, obj.y, z, obj_rpy(2), size, is_last);
          break;
        case VS_OBJECT_COLLECTION_T_SONARCONE:
          draw_sonarcone(self, obj.x, obj.y, z, obj_rpy(2), obj_rpy(1), obj_rpy(0), size, is_last);
          draw_axis(self, obj.x, obj.y, obj.z, obj_rpy(2), obj_rpy(1), obj_rpy(0), size, is_last);
          break;
  }
      }
    }
    glPopAttrib ();
    */
  }
  virtual void clear() {
    elements.clear();
  }
};





// general method for parsing different collection messages, and
// adding to local data structures as well as GUI
template <class MyCollection>
static void on_collection_data(const typename MyCollection::my_vs_collection_t *msg) {
  std::cout << "on_collection_data\n";
/*  RendererCollections *self = (RendererCollections*) user_data;
  collections_t* collections = &self->collections;

  g_mutex_lock(self->collectionsMutex);

  // find object collection, create new one if necessary, update record
  collections_t::iterator collection_it = collections->find(msg->id);
  if (collection_it==collections->end()) {
    MyCollection* collection = new MyCollection(msg->id, msg->name, msg->type, true);
    collections->insert(make_pair(msg->id, collection));
    collection_it = collections->find(msg->id);
    // also add new menu entry for trajectory
    add_checkbox(self, msg->name, msg->id);
  }
  Collection* collection = collection_it->second;

  // update objs
  typename MyCollection::elements_t& elements = dynamic_cast<MyCollection*>(collection)->elements;
  if (msg->reset) {
    collection->clear();
  }
  for (int i=0; i<MyCollection::get_size(msg); i++) {
    MyCollection::copy(msg, i, elements);
  }
  g_mutex_unlock(self->collectionsMutex);

  bot_viewer_request_redraw (self->viewer);
  */
}



























//----------------------------------------------------------------------------


void vtkCollections::on_obj_collection_data(const char* messageData)
{

  int status = vs_object_collection_t_decode (messageData, 0, 1e9, &this->Internal->msg);
  std::cout << "got some collections data " <<  this->Internal->msg.nobjects << "\n";
  on_collection_data<ObjCollection>(&this->Internal->msg);


  if (!status)
  {
    this->Internal->msg.nobjects = 0;
  }else{


    // set transform. 
    // TODO: fix this
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
    //    this->Internal->octd->m_ocTreeTransform[j*4+i] = this->Internal->msg.transform[i][j];
      }
    }

    /*
    std::stringstream datastream;
    datastream.write((const char*) this->Internal->msg.data, this->Internal->msg.nobjects);

    bool fromMessage = true;

    if (fromMessage){
 
      // check if first line valid:
      std::string line;
      std::getline(datastream, line);

      std::string fileHeaderBt = "# Octomap OcTree binary file";
      std::string fileHeaderOt = "# Octomap OcTree file";


    }else{ // read from a file, TODO: move this else where
      //  std::string m_filename = "/home/mfallon/Dropbox/shared/2015-11-octomap/octomap.bt";
      // std::string m_filename = "/home/mfallon/Dropbox/shared/2015-11-octomap/simple_color_tree.ot";
      std::string m_filename = "/home/mfallon/Dropbox/shared/2015-11-octomap/freiburg1_360.ot";

      QString temp = QString(m_filename.c_str());
      QFileInfo fileinfo(temp);



    }
    */
  }
}

//----------------------------------------------------------------------------
void vtkCollections::ReleaseGraphicsResources(vtkWindow *w)
{
  for (size_t i = 0; i < this->Internal->Actors.size(); ++i)
    {
    this->Internal->Actors[i]->ReleaseGraphicsResources(w);
    }
}

//----------------------------------------------------------------------------
int vtkCollections::RenderOpaqueGeometry(vtkViewport *v)
{
//  return -1;

  if (this->Internal->msg.nobjects)
    {
    glPushMatrix();
    glPushAttrib(GL_ENABLE_BIT | GL_POINT_BIT | GL_POLYGON_STIPPLE_BIT |
                 GL_POLYGON_BIT | GL_LINE_BIT | GL_FOG_BIT | GL_LIGHTING_BIT);


    // The goal here is to setup the GL state machine to match what the
    // default libbot viewer uses.  At this point, the VTK graphichs
    // engine may be in some unknown state due to whatever has already been
    // drawn.  In order to set the GL state machine to match libbot, I have
    // reviewed code in:
    //   my_draw() of bot_lcmgl_render/lcmgl_decode.c
    //   render_scene() of bot_vis/viewer.c

    // reset lighting and color flags
    glDisable(GL_LIGHTING);
    glDisable(GL_COLOR_MATERIAL);
    glDisable(GL_LIGHT0);
    glDisable(GL_LIGHT1);
    glDisable(GL_LIGHT2);
    glDisable(GL_LIGHT3);
    glDisable(GL_LIGHT4);
    glDisable(GL_LIGHT5);
    glDisable(GL_LIGHT6);

    // reset lighting model
    float light_model_ambient[] = {0.2, 0.2, 0.2, 1.0};
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, light_model_ambient);

    // reset GL_LIGHT0
    float light0_amb[] = { 0.0, 0.0, 0.0, 1 };
    float light0_dif[] = { 1, 1, 1, 1 };
    float light0_spe[] = { .5, .5, .5, 1 };
    float light0_pos[] = { 100, 100, 100, 0 };
    float spot_exp[] = {0.0};
    float spot_cutoff[] = {180.0};
    float att0[] = {0.0};
    float att1[] = {1.0};
    glLightfv (GL_LIGHT0, GL_AMBIENT, light0_amb);
    glLightfv (GL_LIGHT0, GL_DIFFUSE, light0_dif);
    glLightfv (GL_LIGHT0, GL_SPECULAR, light0_spe);
    glLightfv (GL_LIGHT0, GL_SPOT_EXPONENT, spot_exp);
    glLightfv (GL_LIGHT0, GL_SPOT_CUTOFF, spot_cutoff);
    glLightfv (GL_LIGHT0, GL_CONSTANT_ATTENUATION, att1);
    glLightfv (GL_LIGHT0, GL_LINEAR_ATTENUATION, att1);
    glLightfv (GL_LIGHT0, GL_QUADRATIC_ATTENUATION, att1);

    // don't set the light position, use the position that VTK has already set
    //glLightfv (GL_LIGHT0, GL_POSITION, light0_pos);

    // enable GL_LIGHT0
    glEnable (GL_LIGHT0);

    // this makes the rendering prettier for lines and points
    if (1)
    {
      glEnable(GL_LINE_STIPPLE);
      glEnable(GL_LINE_SMOOTH);
      glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
      glEnable(GL_POINT_SMOOTH);
      glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
    }

    // enable transparency, copied from my_draw() of bot_lcmgl_render/lcmgl_decode.c
    glEnable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);

    //for (std::map<int, OcTreeRecord>::iterator it = this->Internal->m_octrees.begin(); it != this->Internal->m_octrees.end(); ++it) {
    //  it->second.octree_drawer->draw();
    //}

    glPopAttrib ();
    glPopMatrix();
    return 1;
    }

  int count=0;

  for (size_t i = 0; i < this->Internal->Actors.size(); ++i)
    {
    count += this->Internal->Actors[i]->RenderOpaqueGeometry(v);
    }
  
  return count;
}

//----------------------------------------------------------------------------
int vtkCollections::RenderOverlay(vtkViewport *v)
{
  int count=0;

  for (size_t i = 0; i < this->Internal->Actors.size(); ++i)
    {
    count += this->Internal->Actors[i]->RenderOverlay(v);
    }

  return count;
}

//----------------------------------------------------------------------------
int vtkCollections::RenderTranslucentPolygonalGeometry(vtkViewport *v)
{
  int count=0;

  for (size_t i = 0; i < this->Internal->Actors.size(); ++i)
    {
    count += this->Internal->Actors[i]->RenderTranslucentPolygonalGeometry(v);
    }

  return count;
}

//----------------------------------------------------------------------------
int vtkCollections::HasTranslucentPolygonalGeometry()
{
  int result = 0;

  for (size_t i = 0; i < this->Internal->Actors.size(); ++i)
    {
    result |= this->Internal->Actors[i]->HasTranslucentPolygonalGeometry();
    }

  return result;
}

//----------------------------------------------------------------------------
void vtkCollections::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
}
