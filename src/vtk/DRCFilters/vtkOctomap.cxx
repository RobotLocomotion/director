/*=========================================================================

Program:   Visualization Toolkit

Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
All rights reserved.
See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

This software is distributed WITHOUT ANY WARRANTY; without even
the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkOctomap.h"

#include <vtkObjectFactory.h>
#include <vtkSmartPointer.h>
#include <vtkActor.h>

#include <vtkOpenGL.h>

#include <lcmtypes/octomap/raw_t.hpp>
#include <lcmtypes/octomap_raw_t.h>
#include <sstream>


using namespace octomap;

vtkStandardNewMacro(vtkOctomap);


class vtkOctomap::vtkInternal {
public:
  vtkInternal()
    {
      this->msg.length = 0;
    }

  octomap_raw_t msg;

  std::map<int, OcTreeRecord> m_octrees;
  //ViewerWidget* m_glwidget;

  double m_octreeResolution;
  unsigned int m_max_tree_depth;

  std::vector<vtkSmartPointer<vtkActor> > Actors;


};


void vtkOctomap::openTree(std::string filename){

  OcTree* tree = new octomap::OcTree(filename);
  this->addOctree(tree, DEFAULT_OCTREE_ID);

  this->Internal->m_octreeResolution = tree->getResolution();
  //emit changeResolution(this->Internal->m_octreeResolution);
  //setOcTreeUISwitches();

  showOcTree();

  //m_glwidget->resetView();
}

void vtkOctomap::openOcTree(std::string filename){
  AbstractOcTree* tree = AbstractOcTree::read(filename);

  if (tree){
    this->addOctree(tree, DEFAULT_OCTREE_ID);

    this->Internal->m_octreeResolution = tree->getResolution();
    //emit changeResolution(this->Internal->m_octreeResolution);

    //setOcTreeUISwitches();
    showOcTree();
    //m_glwidget->resetView();

    if (tree->getTreeType() == "ColorOcTree"){
      // map color and height map share the same color array and QAction
      // ui.actionHeight_map->setText ("Map color");  // rename QAction in Menu
      this->setColorMode(SceneObject::CM_COLOR_HEIGHT); // enable color view
      // ui.actionHeight_map->setChecked(true);
    }
  }
  else {
    std::cout << "Cannot open OcTree file\n";
    //QMessageBox::warning(this, "File error", "Cannot open OcTree file", QMessageBox::Ok);
  }
}



void vtkOctomap::parseTree(std::string datastream_string){

  std::stringstream datastream(datastream_string);
  OcTree* tree = new octomap::OcTree(1);
  tree->readBinary(datastream);
 
  this->addOctree(tree, DEFAULT_OCTREE_ID);

  this->Internal->m_octreeResolution = tree->getResolution();
  //emit changeResolution(this->Internal->m_octreeResolution);
  //setOcTreeUISwitches();

  showOcTree();

  //m_glwidget->resetView();
}


void vtkOctomap::parseOcTree(std::string datastream_string){

  std::stringstream datastream(datastream_string);

  AbstractOcTree* tree = AbstractOcTree::read(datastream);

  if (tree){
    this->addOctree(tree, DEFAULT_OCTREE_ID);

    this->Internal->m_octreeResolution = tree->getResolution();
    //emit
    //changeResolution(this->Internal->m_octreeResolution);

    //setOcTreeUISwitches();
    showOcTree();
    //m_glwidget->resetView();

    if (tree->getTreeType() == "ColorOcTree"){
      // map color and height map share the same color array and QAction
      // ui.actionHeight_map->setText ("Map color");  // rename QAction in Menu
      this->setColorMode(SceneObject::CM_COLOR_HEIGHT); // enable color view
      // ui.actionHeight_map->setChecked(true);
    }
  }
  else {
    std::cout << "Cannot open OcTree file\n";
    //QMessageBox::warning(this, "File error", "Cannot open OcTree file", QMessageBox::Ok);
  }


}



void vtkOctomap::showOcTree() {

  // update viewer stat
  double minX, minY, minZ, maxX, maxY, maxZ;
  minX = minY = minZ = -10; // min bbx for drawing
  maxX = maxY = maxZ = 10;  // max bbx for drawing
  double sizeX, sizeY, sizeZ;
  sizeX = sizeY = sizeZ = 0.;
  size_t memoryUsage = 0;
  size_t num_nodes = 0;
  size_t memorySingleNode = 0;


  for (std::map<int, OcTreeRecord>::iterator it = this->Internal->m_octrees.begin(); it != this->Internal->m_octrees.end(); ++it) {
    // get map bbx
    double lminX, lminY, lminZ, lmaxX, lmaxY, lmaxZ;
    it->second.octree->getMetricMin(lminX, lminY, lminZ);
    it->second.octree->getMetricMax(lmaxX, lmaxY, lmaxZ);
    // transform to world coords using map origin
    octomap::point3d pmin(lminX, lminY, lminZ);
    octomap::point3d pmax(lmaxX, lmaxY, lmaxZ);
    pmin = it->second.origin.transform(pmin);
    pmax = it->second.origin.transform(pmax);
    lminX = pmin.x(); lminY = pmin.y(); lminZ = pmin.z();
    lmaxX = pmax.x(); lmaxY = pmax.y(); lmaxZ = pmax.z();
    // update global bbx
    if (lminX < minX) minX = lminX;
    if (lminY < minY) minY = lminY;
    if (lminZ < minZ) minZ = lminZ;
    if (lmaxX > maxX) maxX = lmaxX;
    if (lmaxY > maxY) maxY = lmaxY;
    if (lmaxZ > maxZ) maxZ = lmaxZ;
    double lsizeX, lsizeY, lsizeZ;
    // update map stats
    it->second.octree->getMetricSize(lsizeX, lsizeY, lsizeZ);
    if (lsizeX > sizeX) sizeX = lsizeX;
    if (lsizeY > sizeY) sizeY = lsizeY;
    if (lsizeZ > sizeZ) sizeZ = lsizeZ;
    memoryUsage += it->second.octree->memoryUsage();
    num_nodes += it->second.octree->size();
    memorySingleNode = std::max(memorySingleNode, it->second.octree->memoryUsageNode());
  }

  //m_glwidget->setSceneBoundingBox(qglviewer::Vec(minX, minY, minZ), qglviewer::Vec(maxX, maxY, maxZ));

  //if (m_octrees.size()) {
  QString size = QString("%L1 x %L2 x %L3 m^3; %L4 nodes").arg(sizeX).arg(sizeY).arg(sizeZ).arg(unsigned(num_nodes));
  QString memory = QString("Single node: %L1 B; ").arg(memorySingleNode)
            + QString ("Octree: %L1 B (%L2 MB)").arg(memoryUsage).arg((double) memoryUsage/(1024.*1024.), 0, 'f', 3);
  //m_mapMemoryStatus->setText(memory);
  //m_mapSizeStatus->setText(size);
  //}

  //m_glwidget->updateGL();

  // generate cubes -> display
  // timeval start;
  // timeval stop;
  // gettimeofday(&start, NULL);  // start timer
  for (std::map<int, OcTreeRecord>::iterator it = this->Internal->m_octrees.begin(); it != this->Internal->m_octrees.end(); ++it) {
    it->second.octree_drawer->setMax_tree_depth(this->Internal->m_max_tree_depth);
    it->second.octree_drawer->setOcTree(*it->second.octree, it->second.origin, it->second.id);
  }
  //    gettimeofday(&stop, NULL);  // stop timer
  //    double time_to_generate = (stop.tv_sec - start.tv_sec) + 1.0e-6 *(stop.tv_usec - start.tv_usec);
  //    fprintf(stderr, "setOcTree took %f sec\n", time_to_generate);
  //m_glwidget->updateGL();
}



bool vtkOctomap::getOctreeRecord(int id, OcTreeRecord*& otr) {
  std::map<int, OcTreeRecord>::iterator it = this->Internal->m_octrees.find(id);
  if( it != this->Internal->m_octrees.end() ) {
    otr = &(it->second);
    return true;
  }
  else {
    return false;
  }
}

void vtkOctomap::addOctree(octomap::AbstractOcTree* tree, int id, octomap::pose6d origin) {
  // is id in use?
      OcTreeRecord* r;
      bool foundRecord = getOctreeRecord(id, r);
      if (foundRecord && r->octree->getTreeType().compare(tree->getTreeType()) !=0){
        // delete old drawer, create new
        delete r->octree_drawer;
        if (dynamic_cast<OcTree*>(tree)) {
          r->octree_drawer = new OcTreeDrawer();
          //        fprintf(stderr, "adding new OcTreeDrawer for node %d\n", id);
        }
        else if (dynamic_cast<ColorOcTree*>(tree)) {
          r->octree_drawer = new ColorOcTreeDrawer();
        } else{
          OCTOMAP_ERROR("Could not create drawer for tree type %s\n", tree->getTreeType().c_str());
        }

        delete r->octree;
        r->octree = tree;
        r->origin = origin;

      } else if (foundRecord && r->octree->getTreeType().compare(tree->getTreeType()) ==0) {
        // only swap out tree

        delete r->octree;
        r->octree = tree;
        r->origin = origin;
      } else {
        // add new record
        OcTreeRecord otr;
        otr.id = id;
        if (dynamic_cast<OcTree*>(tree)) {
          otr.octree_drawer = new OcTreeDrawer();
          //        fprintf(stderr, "adding new OcTreeDrawer for node %d\n", id);
        }
        else if (dynamic_cast<ColorOcTree*>(tree)) {
          otr.octree_drawer = new ColorOcTreeDrawer();
        } else{
          OCTOMAP_ERROR("Could not create drawer for tree type %s\n", tree->getTreeType().c_str());
        }
        otr.octree = tree;
        otr.origin = origin;
        this->Internal->m_octrees[id] = otr;
        //this->Internal->m_glwidget->addSceneObject(otr.octree_drawer);
      }
}

void vtkOctomap::addOctree(octomap::AbstractOcTree* tree, int id) {
  octomap::pose6d o; // initialized to (0,0,0) , (0,0,0,1) by default
  addOctree(tree, id, o);
}


//----------------------------------------------------------------------------
vtkOctomap::vtkOctomap()
{
  this->Internal = new vtkInternal;
}

//----------------------------------------------------------------------------
vtkOctomap::~vtkOctomap()
{
  delete this->Internal;
}

//----------------------------------------------------------------------------


void vtkOctomap::setAlphaOccupied(double alphaOccupied){
  for (std::map<int, OcTreeRecord>::iterator it = this->Internal->m_octrees.begin(); it != this->Internal->m_octrees.end(); ++it) {
    it->second.octree_drawer->setAlphaOccupied(alphaOccupied);
  }
}


void vtkOctomap::changeTreeDepth(int depth){
  // range check:
  if (depth < 1 || depth > 16)
    return;

  this->Internal->m_max_tree_depth = unsigned(depth);

  if (this->Internal->m_octrees.size() > 0)
    showOcTree();
}

void vtkOctomap::enableOctreeStructure(bool enabled) {
  for (std::map<int, OcTreeRecord>::iterator it = this->Internal->m_octrees.begin(); it != this->Internal->m_octrees.end(); ++it) {
    it->second.octree_drawer->enableOcTree(enabled);
  }
}

void vtkOctomap::enableOcTreeCells(bool enabled){
  for (std::map<int, OcTreeRecord>::iterator it = this->Internal->m_octrees.begin(); it != this->Internal->m_octrees.end(); ++it) {
    it->second.octree_drawer->enableOcTreeCells(enabled);
  }
}

void vtkOctomap::enableFreespace(bool enabled){
  for (std::map<int, OcTreeRecord>::iterator it = this->Internal->m_octrees.begin(); it != this->Internal->m_octrees.end(); ++it) {
    it->second.octree_drawer->enableFreespace(enabled);
  }
}

void vtkOctomap::setColorMode (int colorMode) {
  SceneObject::ColorMode mode = (SceneObject::ColorMode) (colorMode);//SceneObject::CM_COLOR_HEIGHT;
  for (std::map<int, OcTreeRecord>::iterator it = this->Internal->m_octrees.begin(); it != this->Internal->m_octrees.end(); ++it) {
    it->second.octree_drawer->setColorMode(mode);
  }
}

//----------------------------------------------------------------------------


void vtkOctomap::UpdateOctomapData(const char* messageData)
{

  int status = octomap_raw_t_decode (messageData, 0, 1e9, &this->Internal->msg);

  if (!status)
  {
    this->Internal->msg.length = 0;
  }else{

    // set transform. 
    // TODO: fix this
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
    //    this->Internal->octd->m_ocTreeTransform[j*4+i] = this->Internal->msg.transform[i][j];
      }
    }

    std::stringstream datastream;
    datastream.write((const char*) this->Internal->msg.data, this->Internal->msg.length);

    bool fromMessage = true;

    if (fromMessage){
 
      // check if first line valid:
      std::string line;
      std::getline(datastream, line);

      std::string fileHeaderBt = "# Octomap OcTree binary file";
      std::string fileHeaderOt = "# Octomap OcTree file";
      if (line.compare(0,fileHeaderBt.length(), fileHeaderBt) ==0){
        std::cout << "Octomap Binary Message received\n";
        parseTree(datastream.str());
      }else if (line.compare(0,fileHeaderOt.length(), fileHeaderOt) ==0){
        std::cout << "Octomap OcTree Message received\n";
          parseOcTree(datastream.str());
      }else{
        std::cout << line << " was the first line received\n";
        std::cout << "input data format not understood\n";
      }

    }else{ // read from a file, TODO: move this else where
      //  std::string m_filename = "/home/mfallon/Dropbox/shared/2015-11-octomap/octomap.bt";
      // std::string m_filename = "/home/mfallon/Dropbox/shared/2015-11-octomap/simple_color_tree.ot";
      std::string m_filename = "/home/mfallon/Dropbox/shared/2015-11-octomap/freiburg1_360.ot";

      QString temp = QString(m_filename.c_str());
      QFileInfo fileinfo(temp);

      if (fileinfo.suffix() == "bt"){
        openTree(m_filename);
      }
      else if (fileinfo.suffix() == "ot")
      {
        openOcTree(m_filename);
      }

    }
  }
}

//----------------------------------------------------------------------------
void vtkOctomap::ReleaseGraphicsResources(vtkWindow *w)
{
  for (size_t i = 0; i < this->Internal->Actors.size(); ++i)
    {
    this->Internal->Actors[i]->ReleaseGraphicsResources(w);
    }
}

//----------------------------------------------------------------------------
int vtkOctomap::RenderOpaqueGeometry(vtkViewport *v)
{
//  return -1;

  if (this->Internal->msg.length)
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

    for (std::map<int, OcTreeRecord>::iterator it = this->Internal->m_octrees.begin(); it != this->Internal->m_octrees.end(); ++it) {
      it->second.octree_drawer->draw();
    }

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
int vtkOctomap::RenderOverlay(vtkViewport *v)
{
  int count=0;

  for (size_t i = 0; i < this->Internal->Actors.size(); ++i)
    {
    count += this->Internal->Actors[i]->RenderOverlay(v);
    }

  return count;
}

//----------------------------------------------------------------------------
int vtkOctomap::RenderTranslucentPolygonalGeometry(vtkViewport *v)
{
  int count=0;

  for (size_t i = 0; i < this->Internal->Actors.size(); ++i)
    {
    count += this->Internal->Actors[i]->RenderTranslucentPolygonalGeometry(v);
    }

  return count;
}

//----------------------------------------------------------------------------
int vtkOctomap::HasTranslucentPolygonalGeometry()
{
  int result = 0;

  for (size_t i = 0; i < this->Internal->Actors.size(); ++i)
    {
    result |= this->Internal->Actors[i]->HasTranslucentPolygonalGeometry();
    }

  return result;
}

//----------------------------------------------------------------------------
void vtkOctomap::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
}
