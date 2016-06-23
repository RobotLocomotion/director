// TODO:
// - Internal should not be public. Resolve this, perhaps not by passing around a void* to vtkCollections
// - explore if mutex is needed
// - remove need for different msg_point and msg_link
// Missing features from old renderer:
// Renderer other information e.g. text, covariance ellipses

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

#include "lcmtypes/visualization.h"
#include <sstream>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <QString>
#include <QFileInfo>

using namespace std;
using namespace Eigen;


#define bot_to_degrees(rad) ((rad)*180/M_PI)

const bool PARAM_USE_TIME_COLLECTION_DEFAULT = false;
const bool PARAM_FILL_SCANS_DEFAULT = false;

const double PARAM_POINT_WIDTH_POINTS_DEFAULT = 5;//1
const double PARAM_TIME_SCALE_DEFAULT = 10.;
const double PARAM_RANGE_START_DEFAULT = 0.;
const double PARAM_RANGE_END_DEFAULT = 1.;
const double PARAM_POSE_WIDTH_POSES_DEFAULT = 1;
const double PARAM_ALPHA_POINTS_DEFAULT = 1.;
const bool PARAM_COLOR_TIME_DEFAULT = false;

const bool PARAM_USE_TIME_DEFAULT = false;
const bool PARAM_Z_UP_DEFAULT = true;

const bool PARAM_COLOR_AXES = false;

const bool PARAM_SHOW_TOGGLE_DEFAULT = false;

float colors[] = {
    51/255.0, 160/255.0, 44/255.0,
    166/255.0, 206/255.0, 227/255.0,
    178/255.0, 223/255.0, 138/255.0,
    31/255.0, 120/255.0, 180/255.0,
    251/255.0, 154/255.0, 153/255.0,
    227/255.0, 26/255.0, 28/255.0,
    253/255.0, 191/255.0, 111/255.0,
    106/255.0, 61/255.0, 154/255.0,
    255/255.0, 127/255.0, 0/255.0,
    202/255.0, 178/255.0, 214/255.0,
    1.0, 0.0, 0.0, // red
    0.0, 1.0, 0.0, // green
    0.0, 0.0, 1.0, // blue
    1.0, 1.0, 0.0,
    1.0, 0.0, 1.0,
    0.0, 1.0, 1.0,
    0.5, 1.0, 0.0,
    1.0, 0.5, 0.0,
    0.5, 0.0, 1.0,
    1.0, 0.0, 0.5,
    0.0, 0.5, 1.0,
    0.0, 1.0, 0.5,
    1.0, 0.5, 0.5,
    0.5, 1.0, 0.5,
    0.5, 0.5, 1.0,
    0.5, 0.5, 1.0,
    0.5, 1.0, 0.5,
    0.5, 0.5, 1.0
};

const int num_colors = sizeof(colors)/(3*sizeof(float));


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
  vs_link_collection_t msg_link;
  vs_point3d_list_collection_t msg_points;

  collections_t collections;

  bool param_use_time;
  bool param_use_time_collection;
  double param_time_scale;
  double param_range_start;
  double param_range_end;
  bool param_fill_scans;
  double param_alpha_points;
  double param_point_width;
  double param_pose_width;
  bool param_color_time;
  bool param_z_up;

  bool param_color_axes;
  
  bool toggle_onoff;


  int64_t    obj_maxid;
  int64_t    obj_minid;

  std::vector<vtkSmartPointer<vtkActor> > Actors;
};


//----------------------------------------------------------------------------
vtkCollections::vtkCollections()
{
  this->Internal = new vtkInternal;

  this->Internal->param_use_time    = PARAM_USE_TIME_DEFAULT;
  this->Internal->param_use_time_collection    = PARAM_USE_TIME_COLLECTION_DEFAULT;
  this->Internal->param_time_scale  = PARAM_TIME_SCALE_DEFAULT;
  this->Internal->param_range_start = PARAM_RANGE_START_DEFAULT;
  this->Internal->param_range_end   = PARAM_RANGE_END_DEFAULT;
  this->Internal->param_fill_scans = PARAM_FILL_SCANS_DEFAULT;  
  this->Internal->param_alpha_points = PARAM_ALPHA_POINTS_DEFAULT;
  this->Internal->param_color_time = PARAM_COLOR_TIME_DEFAULT;
  this->Internal->param_point_width = PARAM_POINT_WIDTH_POINTS_DEFAULT;
  this->Internal->param_pose_width = PARAM_POSE_WIDTH_POSES_DEFAULT;
  this->Internal->param_z_up = PARAM_Z_UP_DEFAULT;

  this->Internal->param_color_axes = PARAM_COLOR_AXES;

  this->Internal->toggle_onoff = PARAM_SHOW_TOGGLE_DEFAULT;
}

//----------------------------------------------------------------------------
vtkCollections::~vtkCollections()
{
  delete this->Internal;
}

//----------------------------------------------------------------------------

int vtkCollections::getCollectionsSize(){
  collections_t* collections = &this->Internal->collections;
  return collections->size();
}

int vtkCollections::getCollectionsId(int mapIndex ){
  collections_t &collections = this->Internal->collections;
  int counter = 0;
  for (collections_t::iterator it = collections.begin(); it!=collections.end(); it++) {
    if (counter== mapIndex){
      return it->second->id;
    }
    counter++;
  }
  return -1;
}
std::string vtkCollections::getCollectionsName(int mapIndex ){
  collections_t &collections = this->Internal->collections;
  int counter = 0;
  for (collections_t::iterator it = collections.begin(); it!=collections.end(); it++) {
    if (counter== mapIndex){
      return it->second->name;
    }
    counter++;
  }
  return std::string("");
}
int vtkCollections::getCollectionsType(int mapIndex ){
  collections_t &collections = this->Internal->collections;
  int counter = 0;
  for (collections_t::iterator it = collections.begin(); it!=collections.end(); it++) {
    if (counter== mapIndex){
      return it->second->type;
    }
    counter++;
  }
  return -1;
}
bool vtkCollections::getCollectionsShow(int mapIndex ){
  collections_t &collections = this->Internal->collections;
  int counter = 0;
  for (collections_t::iterator it = collections.begin(); it!=collections.end(); it++) {
    if (counter== mapIndex){
      return it->second->show;
    }
    counter++;
  }
  return false;
}

void vtkCollections::setEnabled(int id, bool show){
  collections_t &collections = this->Internal->collections;
  collections_t::iterator it = collections.find(id);
  if (it!=collections.end()) {
    it->second->show = show;
  }
}

void vtkCollections::removeIdFromCollections(int id){
  collections_t &collections = this->Internal->collections;
  collections_t::iterator collection_it = collections.find(id);
  collections.erase (collection_it);
}

void vtkCollections::setRangeStart(double rangeStart){
  this->Internal->param_range_start = rangeStart;
}
void vtkCollections::setRangeEnd(double rangeEnd){
  this->Internal->param_range_end = rangeEnd;
}
void vtkCollections::setAlphaPoints(double alphaPoints){
  this->Internal->param_alpha_points = alphaPoints;
}

void vtkCollections::setFillScans(bool fillScans){
  this->Internal->param_fill_scans = fillScans;
}

void vtkCollections::setPointWidth(double pointWidth){
  this->Internal->param_point_width = pointWidth;
}
void vtkCollections::setPoseWidth(double poseWidth){
  this->Internal->param_pose_width = poseWidth;
}
void vtkCollections::setColorPoses(bool colorPoses){
  this->Internal->param_color_axes = colorPoses;
}

void vtkCollections::setColorByTime(bool colorByTime){
  this->Internal->param_color_time = colorByTime;
}

void vtkCollections::setElevationByTime(bool elevationByTime){
  this->Internal->param_use_time = elevationByTime;
}
void vtkCollections::setElevationByCollection(bool elevationByCollection){
  this->Internal->param_use_time_collection = elevationByCollection;
}
void vtkCollections::setMaxElevation(double maxElevation){
  this->Internal->param_time_scale = maxElevation;
}
void vtkCollections::setShowToggle(){
  // When toggled, set the show property to the toggle value
  // multiple presses of toggle will turn all on or off
  collections_t &collections = this->Internal->collections;
  for (collections_t::iterator it = collections.begin(); it!=collections.end(); it++) {
    it->second->show = this->Internal->toggle_onoff;
  }
  this->Internal->toggle_onoff = !this->Internal->toggle_onoff;
}


// Config for the collections
std::map<int32_t, CollectionConfig> collectionConfig;

// helper function
/**
 * Combines time and collection elevation
 */
double time_elevation(vtkCollections *self, int64_t id, double z, int collid) {

  double time_scale;
  time_scale =self->Internal->param_time_scale * self->Internal->param_pose_width;

  if (!self->Internal->param_use_time && !self->Internal->param_use_time_collection) return z;
  double newz = 0.0;
  if (self->Internal->param_use_time) {
    int64_t min_id = self->Internal->obj_minid;
    int64_t max_id = self->Internal->obj_maxid;

    newz += ((double)(id-min_id) / (double)(max_id-min_id) * time_scale);
  }
  if (self->Internal->param_use_time_collection) {
    newz += collid * time_scale;
  }
  return newz;
}

/**
 * Only look at collection elevation
 */
static double time_elevation_collection(vtkCollections *self, int64_t id, double z, int collid) {
  double time_scale;
  time_scale =self->Internal->param_time_scale * self->Internal->param_pose_width;

  if (!self->Internal->param_use_time_collection) return z;
  double newz = 0.0;
  if (self->Internal->param_use_time_collection) {
    newz += collid * time_scale;
  }
  return newz;
}


void draw_ellipsoid(vtkCollections *self, double x, double y, double z, int n_covs, double* covs, bool is3d) {
  // covs is xx,xy,xt,yy,yt,tt for 2D poses and xx,xy,yy for 2D points
  // for 2D poses we only use the translational part
  // note that z is set to 0.1, so that the ellipsoid actually shows up

  // TODO: add 2D case
  if (is3d) {
    // populate the matrix
    MatrixXd S(3,3);
    S << covs[0], covs[1], covs[2],
         covs[1], covs[3], covs[4],
         covs[2], covs[4], covs[5];

    // eigenvectors and eigenvalues
    EigenSolver<MatrixXd> es(S);
    double eval1 = es.eigenvalues()[0].real();
    double eval2 = es.eigenvalues()[1].real();
    double eval3 = es.eigenvalues()[2].real();
    VectorXd evec1 = es.eigenvectors().col(0).real();
    VectorXd evec2 = es.eigenvectors().col(1).real();
    VectorXd evec3 = es.eigenvectors().col(2).real();

    // draw ellipsoid
    double k = 1.;    // scale factor
    double radius1 = k * sqrt(eval1);
    double radius2 = k * sqrt(eval2);
    double radius3 = k * sqrt(eval3);
    const double max_radius = 6.;
    if (radius1<max_radius && radius2<max_radius && radius3<max_radius) {
      GLUquadricObj *quadric = gluNewQuadric();
      glPushMatrix();
      glTranslated(x,y,z);
      GLdouble rotation[16] = {
        evec1(0), evec1(1), evec1(2), 0.,
        evec2(0), evec2(1), evec2(2), 0.,
        evec3(0), evec3(1), evec3(2), 0.,
        0., 0., 0., 1.
      };
      glMultMatrixd(rotation);
      glScaled(radius1, radius2, radius3);
      gluQuadricDrawStyle(quadric, GLU_FILL);
      gluQuadricNormals(quadric, GLU_SMOOTH);
      gluSphere(quadric, 0.3, 10, 10);
      glPopMatrix();
    }
  }
}

static void draw_tree(vtkCollections *self, double x, double y, double z) {
  GLUquadricObj * quadric = gluNewQuadric (); // todo
  glPushMatrix();
  glTranslatef (x, y, z);
  glColor3f(139./255., 69./255., 19./255.);
  glPushMatrix();
  glTranslatef(0,0,0.01);
  gluCylinder(quadric, 0.2, 0.2, 2.6, 8, 8);
  glPopMatrix();
  glPushMatrix();
  glTranslatef(0,0,4.5);
  glColor3f(0.1,0.7,0.1);
  gluSphere(quadric, 2., 8, 8);
  glPopMatrix();
  glPopMatrix();
}

static void draw_axis(vtkCollections *self, double x, double y, double z, double yaw, double pitch, double roll, double size, bool mark, int id) 
{

  glPushMatrix();
  glPushAttrib(GL_CURRENT_BIT);

  glTranslatef(x, y, z);

  glRotatef(bot_to_degrees(yaw),  0., 0., 1.);
  glRotatef(bot_to_degrees(pitch),0., 1., 0.);
  glRotatef(bot_to_degrees(roll), 1., 0., 0.);

  glBegin(GL_LINES);

  if (self->Internal->param_color_axes){
    GLfloat color[3];
    color[0] = colors[3*(id%num_colors)];
    color[1] = colors[3*(id%num_colors)+1];
    color[2] = colors[3*(id%num_colors)+2];

    glColor3f(color[0],color[1],color[2]); glVertex3f(0.0,0.0,0.0); glVertex3f(size*1.0,0.0,0.0);
    glColor3f(color[0],color[1],color[2]); glVertex3f(0.0,0.0,0.0); glVertex3f(0.0,size*1.0,0.0);
    glColor3f(color[0],color[1],color[2]); glVertex3f(0.0,0.0,0.0); glVertex3f(0.0,0.0,size*1.0);
  }else{
    glColor3f(1.0,0.0,0.0); glVertex3f(0.0,0.0,0.0); glVertex3f(size*1.0,0.0,0.0);
    glColor3f(0.0,1.0,0.0); glVertex3f(0.0,0.0,0.0); glVertex3f(0.0,size*1.0,0.0);
    glColor3f(0.0,0.0,1.0); glVertex3f(0.0,0.0,0.0); glVertex3f(0.0,0.0,size*1.0);
  }

  glEnd();

  if (mark) {
//    glutWireSphere(size*1.5, 5, 5);
  }

  glPopAttrib();
  // todo: reset color?
  glPopMatrix();
}

static void draw_tag(vtkCollections *self, double x, double y, double z, double yaw, double pitch, double roll) {
  glPushMatrix();
  glPushAttrib(GL_CURRENT_BIT);

  glTranslatef(x, y, z);

  glRotatef(bot_to_degrees(yaw),  0., 0., 1.);
  glRotatef(bot_to_degrees(pitch),0., 1., 0.);
  glRotatef(bot_to_degrees(roll), 1., 0., 0.);

  double size = 0.166;
  double h = size / 2.;

  glBegin(GL_QUADS);
#if 1
  double d = size / 8.;
  for (int i=0; i<8; i++) {
    double x = i*d - h;
    for (int j=0; j<8; j++) {
      if (i==0 || i==7 || j==0 || j==7 || (i%2)==(j%2)) {
        double y = j*d - h;
        glVertex3f(  x,   y, 0);
        glVertex3f(  x, y+d, 0);
        glVertex3f(x+d, y+d, 0);
        glVertex3f(x+d,   y, 0);
      }
    }
  }
#else
  glVertex3f(-h,-h,0);
  glVertex3f(-h, h,0);
  glVertex3f( h, h,0);
  glVertex3f( h,-h,0);
#endif
  glEnd();

  glPopAttrib();
  // todo: reset color?
  glPopMatrix();
}

static void draw_triangle(vtkCollections *self, double x, double y, double z, double theta, double size, bool mark) {
  //if (!self->viewer) return;

  glPushMatrix();
  glTranslatef(x, y, z);
  glRotatef(bot_to_degrees(theta),  0., 0., 1.);

  if (mark) {
//    glutWireSphere(size*1.5, 5, 5);
    glPushAttrib(GL_CURRENT_BIT);
    glPushMatrix();
    glScalef(2.0,2.0,1.0);
    glBegin(GL_LINE_LOOP);
    glColor3f(0.9,0.1,0.1); glVertex3f(size,0.0,0.1);
    glColor3f(0.9,0.1,0.1); glVertex3f(-size,size/2.0,0.1);
    glColor3f(0.9,0.1,0.1); glVertex3f(-size,-size/2.0,0.1);
    glEnd();
    glPopMatrix();
    glPopAttrib();
  }

  glBegin(GL_POLYGON);
  glVertex3f(size,0.0,0.0);
  glVertex3f(-size,size/2.0,0.0);
  glVertex3f(-size,-size/2.0,0.0);
  glEnd();

  glPopMatrix();
}

static void draw_equilateral_triangle(vtkCollections *self, double x, double y, double z, double theta, double size, bool mark) {
  //if (!self->viewer) return;

  glPushMatrix();
  glTranslatef(x, y, z);
  //  glRotatef(bot_to_degrees(theta),  0., 0., 1.);

  double r = size;
  double b = -r / sqrt(3.0);
  double h = sqrt(3.0) * r;

  glBegin(GL_LINE_LOOP);
  glVertex3f(r, b, 0.0);
  glVertex3f(-r, b, 0.0);
  glVertex3f(0.0, h+b, 0.0);
  glEnd();

  glPopMatrix();
}

static void draw_hexagon(vtkCollections *self, double x, double y, double z, double theta, double size, bool mark) {
  //if (!self->viewer) return;

  glPushMatrix();
  glTranslatef(x, y, z);
  glRotatef(bot_to_degrees(theta),  0., 0., 1.);
  double r = size;

  glBegin(GL_POLYGON);

  for (int i = 0; i < 6; i++) {
    float a = i*M_PI/3.0;
    glVertex3f(r*cos(a), r*sin(a), 0.0);
  }

  glEnd();

  glPopMatrix();
}


static void draw_camera(vtkCollections *self, double x, double y, double z, double yaw, double pitch, double roll, double size, bool mark) {
  // @todo implement camera rendering

  //if (!self->viewer) return;

  glPushMatrix();
  glTranslatef(x, y, z);
  glRotatef(bot_to_degrees(yaw),  0., 0., 1.);
  glRotatef(bot_to_degrees(pitch),0., 1., 0.);
  glRotatef(bot_to_degrees(roll), 1., 0., 0.);

  // Depth, height and width of pyramid
  float d = 7.0;
  float h = 240.0/540.0 * d;
  float w = 320.0/540.0 * d;

  //glScalef(size,size,size);
  glBegin(GL_LINES);
  // Draw sides
  glVertex3f(0,0,0); glVertex3f(d,  w, h);
  glVertex3f(0,0,0); glVertex3f(d, -w, h);
  glVertex3f(0,0,0); glVertex3f(d,  w, -h);
  glVertex3f(0,0,0); glVertex3f(d, -w, -h);

  // Draw base;
  glVertex3f(d,  w,  h); glVertex3f(d, -w,  h);
  glVertex3f(d, -w,  h); glVertex3f(d, -w, -h);
  glVertex3f(d, -w, -h); glVertex3f(d,  w, -h);
  glVertex3f(d,  w, -h); glVertex3f(d,  w,  h);
  glEnd();

  glPopMatrix();
}

static void draw_sonarcone(vtkCollections *self, double x, double y, double z, double yaw, double pitch, double roll, double size, bool mark) {
  // based off of draw_camera
  //if (!self->viewer) return;

  glPushMatrix();
  glTranslatef(x, y, z);
  glRotatef(bot_to_degrees(yaw),  0., 0., 1.);
  glRotatef(bot_to_degrees(pitch),0., 1., 0.);
  glRotatef(bot_to_degrees(roll), 1., 0., 0.);
  glBegin(GL_LINES);

  double r = 40; // range of the sonar
  double angle = M_PI/2; // 45 degrees sonar cone
  double nlines = 20; // number of lines used

  double w,d,th;
  double h=0; // off axis height - could be used to make a wedge

  // Draw the sides of the cone:
  th = angle/2;
  w = r*sin(th); // left
  d = r*cos(th); // forward
  glVertex3f(0,0,0); glVertex3f(d,  w, h);
  glVertex3f(0,0,0); glVertex3f(d, -w, h);
  
  for (int i=0;i<nlines;i++){
    th = i*angle/nlines -  angle/2 ;
    w = r*sin(th); // left
    d = r*cos(th); // forward
    glVertex3f(d, w,  h); 
    th = i*angle/nlines - angle/2 + angle/nlines;           ;//- 4*M_PI/20;
    w = r*sin(th); // left
    d = r*cos(th); // forward
    glVertex3f(d, w,  h);
  }

  glEnd();
  glPopMatrix();
}



static void draw_tetra(vtkCollections *self, double x, double y, double z, double yaw, double pitch, double roll, double size, bool mark) {
  //if (!self->viewer) return;

  glPushMatrix();
  glTranslatef(x, y, z);
  glRotatef(bot_to_degrees(yaw),  0., 0., 1.);
  glRotatef(bot_to_degrees(pitch),0., 1., 0.);
  glRotatef(bot_to_degrees(roll), 1., 0., 0.);

  if (mark) {
//    glutWireSphere(size*1.5, 5, 5);
  }
  glBegin(GL_POLYGON);
  glVertex3f(size,0.0,0.0);
  glVertex3f(-size,size/2.0,0.0);
  glVertex3f(-size,-size/2.0,0.0);
  glEnd();
  glBegin(GL_POLYGON);
  glVertex3f(size,0.0,0.0);
  glVertex3f(-size,0.0,size/2.0);
  glVertex3f(-size,-size/2.0,0.0);
  glEnd();
  glBegin(GL_POLYGON);
  glVertex3f(size,0.0,0.0);
  glVertex3f(-size,size/2.0,0.0);
  glVertex3f(-size,0.0,size/2.0);
  glEnd();
  glBegin(GL_POLYGON);
  glVertex3f(-size,0.0,size/2.0);
  glVertex3f(-size,size/2.0,0.0);
  glVertex3f(-size,-size/2.0,0.0);
  glEnd();
  // draw outline in black
  glPushAttrib(GL_CURRENT_BIT);
  glColor3f(0,0,0);
  glBegin(GL_LINE_LOOP);
  glVertex3f(size,0.0,0.0);
  glVertex3f(-size,size/2.0,0.0);
  glVertex3f(-size,-size/2.0,0.0);
  glEnd();
  glBegin(GL_LINE_LOOP);
  glVertex3f(size,0.0,0.0);
  glVertex3f(-size,0.0,size/2.0);
  glVertex3f(-size,-size/2.0,0.0);
  glEnd();
  glBegin(GL_LINE_LOOP);
  glVertex3f(size,0.0,0.0);
  glVertex3f(-size,size/2.0,0.0);
  glVertex3f(-size,0.0,size/2.0);
  glEnd();
  glBegin(GL_LINE_LOOP);
  glVertex3f(-size,0.0,size/2.0);
  glVertex3f(-size,size/2.0,0.0);
  glVertex3f(-size,-size/2.0,0.0);
  glEnd();
  glPopAttrib();
  // todo: reset color?
  glPopMatrix();
}

static void draw_square(vtkCollections *self, double x, double y, double z, double theta, double size) {
  //if (!self->viewer) return;

  glPushMatrix();
  glTranslatef(x, y, z);
  glRotatef(bot_to_degrees(theta),0.0,0.0,1.0);
  glBegin(GL_LINE_LOOP);
  glVertex3f(size,size,0.0);
  glVertex3f(-size,size,0.0);
  glVertex3f(-size,-size,0.0);
  glVertex3f(size,-size,0.0);
  glEnd();
  glPopMatrix();
}


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
    vtkCollections *self = (vtkCollections*) _self;
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
        size = size*self->Internal->param_pose_width;

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
          draw_axis (self, obj.x, obj.y, z, obj_rpy(2), obj_rpy(1), obj_rpy(0), size, is_last, id);
          break;
        case VS_OBJECT_COLLECTION_T_TREE:
          draw_tree (self, obj.x, obj.y, z);
          break;
        case VS_OBJECT_COLLECTION_T_TAG:
          draw_tag (self, obj.x, obj.y, z, obj_rpy(2), obj_rpy(1), obj_rpy(0));
          break;
        case VS_OBJECT_COLLECTION_T_CAMERA:
          draw_camera (self, obj.x, obj.y, z, obj_rpy(2), obj_rpy(1), obj_rpy(0), size, is_last);
          draw_axis(self, obj.x, obj.y, obj.z, obj_rpy(2), obj_rpy(1), obj_rpy(0), size, is_last, id);
          break;
        case VS_OBJECT_COLLECTION_T_TRIANGLE:
          draw_equilateral_triangle (self, obj.x, obj.y, z, obj_rpy(2), size, is_last );
          break;
        case VS_OBJECT_COLLECTION_T_HEXAGON:
          draw_hexagon (self, obj.x, obj.y, z, obj_rpy(2), size, is_last);
          break;
        case VS_OBJECT_COLLECTION_T_SONARCONE:
          draw_sonarcone(self, obj.x, obj.y, z, obj_rpy(2), obj_rpy(1), obj_rpy(0), size, is_last);
          draw_axis(self, obj.x, obj.y, obj.z, obj_rpy(2), obj_rpy(1), obj_rpy(0), size, is_last, id);
          break;
        }
      }
    }
    glPopAttrib ();
  }
  virtual void clear() {
    elements.clear();
  }
};

class LinkCollection : public Collection {
public:
  typedef vs_link_collection_t my_vs_collection_t;
  typedef vs_link_t my_vs_t;

  // index can be time stamp (64 bit!)
  typedef map<int64_t, my_vs_t> elements_t;

  elements_t elements;

  static int get_size(const my_vs_collection_t *msg) {
    return msg->nlinks;
  }

  static void copy(const my_vs_collection_t *msg, int i, elements_t& dst_map) {
    dst_map[msg->links[i].id] = msg->links[i];
  }

  LinkCollection(int id, string name, int type, bool show) : Collection(id, name, type, show) {}
  virtual ~LinkCollection() {}

  virtual void draw(void *_self, int64_t range_start, int64_t range_end) {
    vtkCollections *self = (vtkCollections*) _self;

    CollectionConfig & config = collectionConfig[id];
    bool isconf = config.is_configured();

    glEnable(GL_DEPTH_TEST);

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

    for (elements_t::iterator it = elements.begin(); it != elements.end(); it++) {
      vs_link_t& link = it->second;
      collections_t::iterator collection_it1 = self->Internal->collections.find(link.collection1);
      collections_t::iterator collection_it2 = self->Internal->collections.find(link.collection2);
      if (collection_it1 != self->Internal->collections.end()
          && collection_it2 != self->Internal->collections.end()) {
        ObjCollection::elements_t& objs1 = ((ObjCollection*)collection_it1->second)->elements;
        ObjCollection::elements_t::iterator it1 = objs1.find(link.id1);
        ObjCollection::elements_t& objs2 = ((ObjCollection*)collection_it2->second)->elements;
        ObjCollection::elements_t::iterator it2 = objs2.find(link.id2);
        if (it1 != objs1.end() && it2 != objs2.end()) {
          vs_object_t& obj1 = it1->second;
          vs_object_t& obj2 = it2->second;
          // only draw if at least one end point is within the range
          if ((obj1.id>=range_start && obj1.id<=range_end)
              || (obj2.id>=range_start && obj2.id<=range_end)) {
            glColor3fv(color);

            glBegin(GL_LINES);
            double z1 = time_elevation(self, obj1.id, obj1.z, collection_it1->first);
            glVertex3f(obj1.x, obj1.y, z1);
            double z2 = time_elevation(self, obj2.id, obj2.z, collection_it2->first);
            glVertex3f(obj2.x, obj2.y, z2);
            glEnd();
          }
        }
      }
    }
  }
  virtual void clear() {
    elements.clear();
  }
};


class PointsCollection : public Collection {

  // Specifies how the point cloud should be rendered.
  // It can be one of the types in vs_points_collection_t
  GLenum mode;

public:
  typedef vs_point3d_list_collection_t my_vs_collection_t;
  
  typedef struct _my_vs_t{
    _my_vs_t() : entries(0), colors(0), normals(0), ncolors(0), nnormals(0), npoints(0) {}
    int32_t collection_id;
    int64_t element_id;
    int32_t ncolors;
    int32_t nnormals;
    int32_t npoints;
    float* entries;
    float* colors;
    float* normals;
  } my_vs_t;
  
  // index can be time stamp (64 bit!)
  typedef map<int64_t, my_vs_t> elements_t;

  elements_t elements;

  PointsCollection(int id, string name, int type, bool show) : Collection(id, name, type, show)
  {
    if (type == VS_POINT3D_LIST_COLLECTION_T_POINT) mode = GL_POINTS;
    else if (type == VS_POINT3D_LIST_COLLECTION_T_POINT) mode = GL_LINE_STRIP;
    else if (type == VS_POINT3D_LIST_COLLECTION_T_LINE_STRIP) mode = GL_LINE_STRIP;
    else if (type == VS_POINT3D_LIST_COLLECTION_T_LINE_LOOP) mode = GL_LINE_LOOP;
    else if (type == VS_POINT3D_LIST_COLLECTION_T_LINES) mode = GL_LINES;
    else if (type == VS_POINT3D_LIST_COLLECTION_T_TRIANGLE_STRIP) mode = GL_TRIANGLE_STRIP;
    else if (type == VS_POINT3D_LIST_COLLECTION_T_TRIANGLE_FAN) mode = GL_TRIANGLE_FAN;
    else if (type == VS_POINT3D_LIST_COLLECTION_T_TRIANGLES) mode = GL_TRIANGLES;
    else if (type == VS_POINT3D_LIST_COLLECTION_T_QUAD_STRIP) mode = GL_QUAD_STRIP;
    else if (type == VS_POINT3D_LIST_COLLECTION_T_QUADS) mode = GL_QUADS;
    else if (type == VS_POINT3D_LIST_COLLECTION_T_POLYGON) mode = GL_POLYGON;
  }
  
  virtual ~PointsCollection() {
    clear();
  }

  static int get_size(const my_vs_collection_t *msg) {
    return msg->nlists;
  }

  static void copy(const my_vs_collection_t *msg, int i, elements_t& dst_map) {
    my_vs_t& dst = dst_map[msg->point_lists[i].id];

    dst.collection_id = msg->point_lists[i].collection;
    dst.element_id = msg->point_lists[i].element_id;
    dst.ncolors = msg->point_lists[i].ncolors;
    dst.nnormals = msg->point_lists[i].nnormals;
    dst.npoints = msg->point_lists[i].npoints;
    delete [] dst.entries;
    delete [] dst.colors;
    delete [] dst.normals;

    vs_point3d_list_t* point_list = &(msg->point_lists[i]);

    // Store the origin allows us to do fan rendering (e.g. triangle fan)
    dst.colors = NULL;
    dst.normals = NULL;
    dst.entries = new float[point_list->npoints*3 + 3];

    dst.entries[0] = 0;
    dst.entries[1] = 0;
    dst.entries[2] = 0;

    if (point_list->ncolors == point_list->npoints) {
      dst.colors = new float[point_list->ncolors*4 + 4];
      dst.colors[0] = 0;
      dst.colors[1] = 0;
      dst.colors[2] = 0;
      dst.colors[3] = 0;
    }

    if (point_list->nnormals == point_list->npoints) {
      dst.normals = new float[point_list->nnormals*3 + 3];
      dst.normals[0] = 0;
      dst.normals[1] = 0;
      dst.normals[2] = 0;
    }

    // TODO: now we should be able to memcopy some of the data
    //       at least the float values
    for (int k=0; k<point_list->npoints; k++) {
      dst.entries[3*k + 0 + 3] = point_list->points[k].x;
      dst.entries[3*k + 1 + 3] = point_list->points[k].y;
      dst.entries[3*k + 2 + 3] = point_list->points[k].z;

      if (dst.colors)
      {
        dst.colors[4*k + 0 + 4] = point_list->colors[k].r;
        dst.colors[4*k + 1 + 4] = point_list->colors[k].g;
        dst.colors[4*k + 2 + 4] = point_list->colors[k].b;
        dst.colors[4*k + 3 + 4] = 0.5;
      }
      if (dst.normals)
      {
        dst.normals[3*k + 0 + 3] = point_list->normals[k].x;
        dst.normals[3*k + 1 + 3] = point_list->normals[k].y;
        dst.normals[3*k + 2 + 3] = point_list->normals[k].z;
      }
    }
  }

  virtual void draw(void *_self, int64_t range_start, int64_t range_end) {
    vtkCollections *self = (vtkCollections*) _self;
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnableClientState(GL_VERTEX_ARRAY);
    glPointSize((GLfloat)self->Internal->param_point_width);

    // Perform two passes.
    //  1. Fill in scans if enabled
    //  2. Draw points
    for (int pass=0; pass<=1; ++pass) {
    for (elements_t::iterator it = elements.begin(); it != elements.end(); it++) {
      my_vs_t & element = it->second;
      float* entries = it->second.entries;
      float* colors = it->second.colors;

      if (colors) {
        glEnableClientState(GL_COLOR_ARRAY);
        // @todo this could be done more efficiently
        //       if we update only when the alpha changes
        for (int k=0; k<element.ncolors;k++) {
          colors[4*k + 3 + 4] = self->Internal->param_alpha_points;
        }
      }

      collections_t::iterator collection_it = self->Internal->collections.find(element.collection_id);
      if (collection_it != self->Internal->collections.end()) {
        ObjCollection* objs_ptr = dynamic_cast<ObjCollection*>(collection_it->second);
        if (objs_ptr == 0) continue;
        ObjCollection::elements_t& objs = objs_ptr->elements;
        ObjCollection::elements_t::iterator obj_it = objs.find(element.element_id);
        if (obj_it != objs.end()) {
          vs_object_t& obj = obj_it->second;
          if (obj.id>=range_start && obj.id<=range_end) {
            glPushMatrix();
            double z = time_elevation_collection(self, obj.id, obj.z, collection_it->first);
            float* rgb = &::colors[3*(id%num_colors)];
            float* rgb4 = &::colors[3*(id%num_colors+1)];
            float rgb2[] = {1,1,1};
            float rgb3[] = {1,0,0};
            float colmix = 1.0;
            if (self->Internal->param_color_time) {
              colmix = (double)(obj.id-self->Internal->obj_minid) / (double)(self->Internal->obj_maxid - self->Internal->obj_minid);
            }

            float rgbmix[3];
            rgbmix[0] =rgb2[0]*colmix+(1-colmix)*rgb3[0];
            rgbmix[1] =rgb2[1]*colmix+(1-colmix)*rgb3[1];
            rgbmix[2] =rgb2[2]*colmix+(1-colmix)*rgb3[2];

            float alpha = self->Internal->param_alpha_points; 

            // Retrive euler angles and reverse the order to get rpy:
            // Corresponds to a snippet of code from Matt Antone. (AffordanceUpdater.cpp in map server)
            Eigen::Vector3d obj_ypr = Eigen::Matrix3d(Eigen::Quaterniond(obj.qw, obj.qx, obj.qy, obj.qz)).eulerAngles(2,1,0);
            Eigen::Vector3d obj_rpy = Eigen::Vector3d( obj_ypr[2], obj_ypr[1], obj_ypr[0]);

            glVertexPointer(3, GL_FLOAT, 0, entries);
            if (colors) glColorPointer(4, GL_FLOAT, 0, colors);
            if (pass==0 && self->Internal->param_fill_scans) {
//              if (!colors) glColor4f(rgbmix[0],rgbmix[1],rgbmix[2],1);
//              glDrawArrays(GL_TRIANGLE_FAN, 0, element.npoints+1);
                glPushMatrix();

                glTranslatef(obj.x, obj.y, 0.9*z);

                glRotatef(bot_to_degrees(obj_rpy(2)),0.0,0.0,1.0);
                glRotatef(bot_to_degrees(obj_rpy(1)),0.0,1.0,0.0);
                glRotatef(bot_to_degrees(obj_rpy(0)),1.0,0.0,0.0);

                glColor4f(255,255,155,1.0);
                glLineWidth((GLfloat)self->Internal->param_point_width);

                float line_fan[2*element.npoints*3];
                for (int i=0; i<element.npoints; ++i) {
                   int pos = i*2*3;
                   if ((  entries[3 + i*3]*entries[3 + i*3] 
                        + entries[3 + i*3+1]*entries[3 + i*3+1] 
                        + entries[3 + i*3+2]*entries[3 + i*3+2]) < 30*30)
                   {
                       line_fan[pos    ] = 0; //entries[0];
                       line_fan[pos + 1] = 0; //entries[1];
                       line_fan[pos + 2] = 0; //entries[2];
                       line_fan[pos + 3] = entries[3 + i*3];
                       line_fan[pos + 4] = entries[3 + i*3 + 1];
                       line_fan[pos + 5] = entries[3 + i*3 + 2];
                   }
                }

                glVertexPointer(3, GL_FLOAT, 0, line_fan);
                glDrawArrays(GL_LINES, 0, 2*element.npoints);

                glVertexPointer(3, GL_FLOAT, 0, entries);
                glPopMatrix();
            }

            if (self->Internal->param_fill_scans) {
              rgb[0] = 0;
              rgb[1] = 0;
              rgb[2] = 0;
            }

            glTranslatef(obj.x, obj.y, z);
            glRotatef(bot_to_degrees(obj_rpy(2)),0.0,0.0,1.0);
            glRotatef(bot_to_degrees(obj_rpy(1)),0.0,1.0,0.0);
            glRotatef(bot_to_degrees(obj_rpy(0)),1.0,0.0,0.0);

            rgbmix[0] = rgb[0]*colmix+(1-colmix)*rgb4[0];
            rgbmix[1] = rgb[1]*colmix+(1-colmix)*rgb4[1];
            rgbmix[2] = rgb[2]*colmix+(1-colmix)*rgb4[2];

            if (!colors) glColor4f(rgbmix[0],rgbmix[1],rgbmix[2],alpha);
            if (pass==1) glDrawArrays(mode, 1, element.npoints);
            glPopMatrix();
          }
        }
      }
      if (colors) glDisableClientState(GL_COLOR_ARRAY);
    } // for each (element)
    } // for each (pass)
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisable(GL_BLEND);
    glPopAttrib();
  }

  virtual void clear() {
    for (elements_t::iterator it=elements.begin();it!=elements.end();++it) {
      my_vs_t& dst = it->second;
      delete [] dst.entries;
      delete [] dst.colors;
      delete [] dst.normals;
    }
    elements.clear();
  }
};







// general method for parsing different collection messages, and
// adding to local data structures as well as GUI
template <class MyCollection>
void vtkCollections::on_collection_data(const typename MyCollection::my_vs_collection_t *msg) {
  //  RendererCollections *self = (RendererCollections*) user_data;
  collections_t* collections = &this->Internal->collections;

  //  g_mutex_lock(self->collectionsMutex); 
  //std::cout << "insert new data into " << msg->name << "\n";

  // find object collection, create new one if necessary, update record
  collections_t::iterator collection_it = collections->find(msg->id);
  if (collection_it==collections->end()) {
    MyCollection* collection = new MyCollection(msg->id, msg->name, msg->type, true);
    collections->insert(make_pair(msg->id, collection));
    collection_it = collections->find(msg->id);
    // also add new menu entry for trajectory
    //add_checkbox(self, msg->name, msg->id);
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
  //  g_mutex_unlock(self->collectionsMutex);
  //bot_viewer_request_redraw (self->viewer);
}



//----------------------------------------------------------------------------
void vtkCollections::on_obj_collection_data(const char* messageData)
{
  int status = vs_object_collection_t_decode (messageData, 0, 1e9, &this->Internal->msg);
  on_collection_data<ObjCollection>(&this->Internal->msg);
}

void vtkCollections::on_link_collection_data(const char* messageData)
{
  int status = vs_link_collection_t_decode (messageData, 0, 1e9, &this->Internal->msg_link);
  on_collection_data<LinkCollection>(&this->Internal->msg_link);
}

void vtkCollections::on_points_collection_data(const char* messageData)
{
  int status = vs_point3d_list_collection_t_decode (messageData, 0, 1e9, &this->Internal->msg_points);
  on_collection_data<PointsCollection>(&this->Internal->msg_points);
}

void vtkCollections::on_reset_collections_data(const char* messageData)
{
  //int status = vs_reset_collections_t_decode (messageData, 0, 1e9, &this->Internal->msg_points);

  collections_t &collections = this->Internal->collections;
  for (collections_t::iterator it = collections.begin(); it!=collections.end(); it++) {
    Collection* collection = it->second;
    collection->clear();
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


// Update min max values (only used ids from collections that are being displayed
void vtkCollections::calculate_ranges(int64_t& range_start, int64_t& range_end) {
  bool initialized = false;

  for (collections_t::iterator collection_it = this->Internal->collections.begin(); collection_it != this->Internal->collections.end(); collection_it++) {
    // ObjCollection?
    ObjCollection* obj_col = dynamic_cast<ObjCollection*>(collection_it->second);
    if (obj_col != NULL) {
      ObjCollection::elements_t& objs = obj_col->elements;
      if (!initialized) {
        this->Internal->obj_minid = this->Internal->obj_maxid = objs.begin()->second.id;
        initialized = true;
      }
      for (ObjCollection::elements_t::iterator it = objs.begin(); it != objs.end(); it++) {
        vs_object_t& obj = it->second;
        if (it==objs.begin()) {
          obj_col->maxid = obj.id;
        }
        if (obj.id > this->Internal->obj_maxid) this->Internal->obj_maxid = obj.id;
        if (obj.id < this->Internal->obj_minid) this->Internal->obj_minid = obj.id;
        if (obj.id > obj_col->maxid) obj_col->maxid = obj.id;
      }
    }
  }
  double range = (double)(this->Internal->obj_maxid - this->Internal->obj_minid);
  range_start = this->Internal->obj_minid + (int64_t)(range*this->Internal->param_range_start);
  range_end   = this->Internal->obj_minid + (int64_t)(range*this->Internal->param_range_end);
}


//----------------------------------------------------------------------------
int vtkCollections::RenderOpaqueGeometry(vtkViewport *v)
{
//  return -1;

  if (this->Internal->msg.nobjects)
    {

      glPushAttrib (GL_DEPTH_BUFFER_BIT | GL_ENABLE_BIT | GL_LINE_BIT);
      glEnable (GL_DEPTH_TEST);
      glDepthFunc (GL_LESS);
      glDisable (GL_LIGHTING);
      glLineWidth (2);

      glPushMatrix();

      //if (!self->param_z_up) {
        // Viewer is (x,y,z) = (forward,left,up) coordinates
        //    Collections are in (forward,right,down) coordinates
        //    We rotate 180 around x-axis
      //  glRotatef(180.0,1.0,0.0,0.0);
      //}

      /// @todo have GROUND_LEVEL configurable
      //glTranslatef(0.,0., GROUND_LEVEL);

      int64_t range_start;
      int64_t range_end;
      calculate_ranges(range_start, range_end);

      for (collections_t::iterator collection_it = this->Internal->collections.begin(); collection_it != this->Internal->collections.end(); collection_it++) {
        Collection* collection = collection_it->second;
        if (collection->show) {
          collection->draw(this, range_start, range_end);
        }
      }

      glPopMatrix ();

      glPopAttrib ();

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
