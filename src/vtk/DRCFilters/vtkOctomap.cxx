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

#include <octomap/octomap.h>
//#include <octovis/SceneObject.h>
//#include <octovis/OcTreeRecord.h>
#include <sstream>

//----------------------------------------------------------------------------
// random number between [0, 1)
static inline float _randf()
{
    return ((float) rand()) / (RAND_MAX + 1.0);
}

/** Given an array of colors, a palette is created that linearly interpolates through all the colors. **/
static void color_util_build_color_table(double color_palette[][3], int palette_size, float lut[][3], int lut_size)
{
    for (int idx = 0; idx < lut_size; idx++) {
        double znorm = ((double) idx) / lut_size;

        int color_index = (palette_size - 1) * znorm;
        double alpha = (palette_size - 1) * znorm - color_index;

        for (int i = 0; i < 3; i++) {
            lut[idx][i] = color_palette[color_index][i] * (1.0 - alpha) + color_palette[color_index+1][i]*alpha;
        }
    }
}

#define JET_COLORS_LUT_SIZE 1024
static float jet_colors[JET_COLORS_LUT_SIZE][3];
static int jet_colors_initialized = 0;

static void init_color_table_jet()
{
    double jet[][3] = {{ 0,   0,   1 },
                       { 0,  .5,  .5 },
                       { .8, .8,   0 },
                       { 1,   0,   0 }};

    color_util_build_color_table(jet, sizeof(jet)/(sizeof(double)*3), jet_colors, JET_COLORS_LUT_SIZE);
    jet_colors_initialized = 1;
}

float *bot_color_util_jet(double v)
{
    if (!jet_colors_initialized)
        init_color_table_jet();

    v = fmax(0, v);
    v = fmin(1, v);

    int idx = (JET_COLORS_LUT_SIZE - 1) * v;
    return jet_colors[idx];
}




namespace octomap {

OcTreeDrawer::OcTreeDrawer() :
    m_occupiedThresSize(0), m_freeThresSize(0), m_occupiedSize(0), m_freeSize(0), m_selectionSize(0),
        octree_grid_vertex_size(0), m_alphaOccupied(0.8)
{
  m_octree_grid_vis_initialized = false;
  m_drawOccupied = false;
  m_drawOcTreeGrid = false;
  m_drawFree = false;
  m_drawSelection = true;

  m_heightColorMode = false;
  m_zMin = -1;
  m_zMax = 12;

  m_occupiedArray = NULL;
  m_freeArray = NULL;
  m_occupiedThresArray = NULL;
  m_freeThresArray = NULL;
  m_occupiedColorArray = NULL;
  m_occupiedThresColorArray = NULL;
  m_selectionArray = NULL;
}

OcTreeDrawer::~OcTreeDrawer()
{
  clear();
}

void OcTreeDrawer::setAlphaOccupied(double alpha)
{
  m_alphaOccupied = alpha;
}

void OcTreeDrawer::setOcTree(const AbstractOcTree& tree, const pose6d& origin, int map_id_) {
//void OcTreeDrawer::setOcTree(const octomap::OcTree& octree, const pose6d& origin, int map_id_)//, double minDrawZ, double maxDrawZ)

    const OcTree& octree = (const OcTree&) tree;
    this->map_id = map_id_;

    // save origin used during cube generation
    this->initial_origin = octomap::pose6d(octomap::point3d(0,0,0), origin.rot());

    // origin is in global coords
    this->origin = origin;
    
    // maximum size to prevent crashes on large maps: (should be checked in a better way than a constant)
    bool showAll = (octree.size() < 5 * 1e6);
    bool uses_origin = ( (origin.rot().x() != 0.) && (origin.rot().y() != 0.)
        && (origin.rot().z() != 0.) && (origin.rot().u() != 1.) );

    // walk the tree one to find the number of nodes in each category
    // (this is used to set up the OpenGL arrays)
    // TODO: this step may be left out, if we maintained the GLArrays in std::vectors instead...
    unsigned int cnt_occupied(0), cnt_occupied_thres(0), cnt_free(0), cnt_free_thres(0);
    for(OcTree::tree_iterator it = octree.begin_tree(this->m_max_tree_depth),
            end=octree.end_tree(); it!= end; ++it) {
      if (it.isLeaf()) { 
        if (octree.isNodeOccupied(*it)){ // occupied voxels
          if (octree.isNodeAtThreshold(*it)) ++cnt_occupied_thres;
          else                               ++cnt_occupied;
        }
        else if (showAll) { // freespace voxels
          if (octree.isNodeAtThreshold(*it)) ++cnt_free_thres;
          else                               ++cnt_free;
        }
      }        
    }    
    // setup GL arrays for cube quads and cube colors
    initGLArrays(cnt_occupied      , m_occupiedSize     , &m_occupiedArray     , &m_occupiedColorArray);
    initGLArrays(cnt_occupied_thres, m_occupiedThresSize, &m_occupiedThresArray, &m_occupiedThresColorArray);
    initGLArrays(cnt_free          , m_freeSize         , &m_freeArray, NULL);
    initGLArrays(cnt_free_thres    , m_freeThresSize    , &m_freeThresArray, NULL);

    double minX, minY, minZ, maxX, maxY, maxZ;
    octree.getMetricMin(minX, minY, minZ);
    octree.getMetricMax(maxX, maxY, maxZ);

    // set min/max Z for color height map
    m_zMin = minZ;
    m_zMax = maxZ;

    std::vector<octomath::Vector3> cube_template;
    initCubeTemplate(origin, cube_template);

    unsigned int idx_occupied(0), idx_occupied_thres(0), idx_free(0), idx_free_thres(0);
    unsigned int color_idx_occupied(0), color_idx_occupied_thres(0);

    m_grid_voxels.clear();
    OcTreeVolume voxel; // current voxel, possibly transformed 
    for(OcTree::tree_iterator it = octree.begin_tree(this->m_max_tree_depth),
            end=octree.end_tree(); it!= end; ++it) {

      if (it.isLeaf()) { // voxels for leaf nodes
        if (uses_origin) 
          voxel = OcTreeVolume(origin.rot().rotate(it.getCoordinate()), it.getSize());
        else 
          voxel = OcTreeVolume(it.getCoordinate(), it.getSize());
        
        if (octree.isNodeOccupied(*it)){ // occupied voxels
          if (octree.isNodeAtThreshold(*it)) {
            idx_occupied_thres = generateCube(voxel, cube_template, idx_occupied_thres, &m_occupiedThresArray);
            color_idx_occupied_thres = setCubeColorHeightmap(voxel, color_idx_occupied_thres, &m_occupiedThresColorArray);
          }
          else {
            idx_occupied = generateCube(voxel, cube_template, idx_occupied, &m_occupiedArray);
            color_idx_occupied = setCubeColorHeightmap(voxel, color_idx_occupied, &m_occupiedColorArray);
          }
        }
        else if (showAll) { // freespace voxels
          if (octree.isNodeAtThreshold(*it)) {
            idx_free_thres = generateCube(voxel, cube_template, idx_free_thres, &m_freeThresArray);
          }
          else {
            idx_free = generateCube(voxel, cube_template, idx_free, &m_freeArray);
          }
        }
      }
      
      else { // inner node voxels (for grid structure only)
        if (showAll) {
          if (uses_origin)
            voxel = OcTreeVolume(origin.rot().rotate(it.getCoordinate()), it.getSize());
          else
            voxel = OcTreeVolume(it.getCoordinate(), it.getSize());
          m_grid_voxels.push_back(voxel);
        }
      }      
    } // end for all voxels

    m_octree_grid_vis_initialized = false;

    if(m_drawOcTreeGrid)
      initOctreeGridVis();


/*
  octree.getMetricMin(minX, minY, minZ);
  octree.getMetricMax(maxX, maxY, maxZ);
  //printf("map bounds: [%.2f, %.2f, %.2f] - [%.2f, %.2f, %.2f]\n", minX, minY, minZ, maxX, maxY, maxZ);

  std::list<octomap::OcTreeVolume> occupiedThresVoxels;
  std::list<octomap::OcTreeVolume> freeThresVoxels;
  std::list<octomap::OcTreeVolume> occupiedVoxels;
  std::list<octomap::OcTreeVolume> freeVoxels;

  if (m_drawOccupied) {
    int numremoved = 0;
    for (octomap::OcTree::tree_iterator it = octree.begin_tree(m_max_tree_depth), end=octree.end_tree(); it!= end; ++it) {
      if (it.isLeaf()) { // leaf nodes only, to be removed if wanting to include inner nodes
        octomap::OcTreeVolume voxel = OcTreeVolume(it.getCoordinate(), it.getSize());

        if (octree.isNodeOccupied(*it)) {
          if (octree.isNodeAtThreshold(*it)) {
            occupiedThresVoxels.push_back(voxel);
          } else {
            occupiedVoxels.push_back(voxel);
          }
        }
      }
    }

    int size1 = occupiedVoxels.size();
    int size2 = occupiedThresVoxels.size();
    for (std::list<octomap::OcTreeVolume>::iterator it = occupiedVoxels.begin(); it != occupiedVoxels.end();) {
      if (it->first.z() > maxDrawZ || it->first.z() < minDrawZ) {
        it = occupiedVoxels.erase(it);
        numremoved++;
      }
      else
        it++;
    }
    for (std::list<octomap::OcTreeVolume>::iterator it = occupiedThresVoxels.begin(); it != occupiedThresVoxels.end();
        ) {
      if (it->first.z() > maxDrawZ || it->first.z() < minDrawZ) {
        it = occupiedThresVoxels.erase(it);
        numremoved++;
      }
      else
        it++;
    }
    int size3 = occupiedVoxels.size();
    int size4 = occupiedThresVoxels.size();
    //printf("thresh=[%f,%f], numremoved=%d, sizes %d %d %d %d\n", minDrawZ, maxDrawZ, numremoved, size1, size2, size3,
    //    size4);
  }

  if (m_drawFree) {
    for (octomap::OcTree::tree_iterator it = octree.begin_tree(m_max_tree_depth), end=octree.end_tree(); it!= end; ++it) {
      if (it.isLeaf()) { // leaf nodes only, to be removed if wanting to include inner nodes
        octomap::OcTreeVolume voxel = OcTreeVolume(it.getCoordinate(), it.getSize());

        if (!octree.isNodeOccupied(*it)) {
          if (octree.isNodeAtThreshold(*it)) {
            freeThresVoxels.push_back(voxel);
          } else {
            freeVoxels.push_back(voxel);
          }
        }
      }
    }
    int numremoved = 0;
    int size1 = freeVoxels.size();
    int size2 = freeThresVoxels.size();
    for (std::list<octomap::OcTreeVolume>::iterator it = freeVoxels.begin(); it != freeVoxels.end();) {
      if (it->first.z() > maxDrawZ || it->first.z() < minDrawZ) {
        it = freeVoxels.erase(it);
        numremoved++;
      }
      else
        it++;
    }
    for (std::list<octomap::OcTreeVolume>::iterator it = freeThresVoxels.begin(); it != freeThresVoxels.end();
        ) {
      if (it->first.z() > maxDrawZ || it->first.z() < minDrawZ) {
        it = freeThresVoxels.erase(it);
        numremoved++;
      }
      else
        it++;
    }
    int size3 = freeVoxels.size();
    int size4 = freeThresVoxels.size();
    //printf("free numremoved=%d, sizes %d %d %d %d\n", numremoved, size1, size2, size3,
    //    size4);
  }

  m_octree_grid_vis_initialized = false;
//  if (m_drawOcTreeGrid && octree.size() < 10 * 1e6) {
//    octree.getVoxels(m_grid_voxels, m_max_tree_depth - 1); // octree structure not drawn at lowest level
//    initOctreeGridVis();
//  }

  // initialize visualization:
  generateCubes(occupiedThresVoxels, &m_occupiedThresArray, m_occupiedThresSize, &m_occupiedThresColorArray);
  generateCubes(freeThresVoxels, &m_freeThresArray, m_freeThresSize);

  generateCubes(occupiedVoxels, &m_occupiedArray, m_occupiedSize, &m_occupiedColorArray);
  generateCubes(freeVoxels, &m_freeArray, m_freeSize);
*/
}



  void OcTreeDrawer::initGLArrays(const unsigned int& num_cubes,
                                  unsigned int& glArraySize,
                                  GLfloat*** glArray, GLfloat** glColorArray) {

    clearCubes(glArray, glArraySize, glColorArray);

    // store size of GL arrays for drawing
    glArraySize = num_cubes * 4 * 3;

    // allocate cube arrays, 6 quads per cube
    *glArray = new GLfloat* [6];
    for (unsigned i = 0; i<6; ++i){
      (*glArray)[i] = new GLfloat[glArraySize];
    }
    // setup quad color array, if given
    if (glColorArray != NULL)
      *glColorArray = new GLfloat[glArraySize * 4 *4];
  }

  void OcTreeDrawer::initCubeTemplate(const octomath::Pose6D& origin,
                                      std::vector<octomath::Vector3>& cube_template) {
    cube_template.clear();
    cube_template.reserve(24);

    cube_template.push_back(octomath::Vector3( 1, 1,-1));
    cube_template.push_back(octomath::Vector3( 1,-1,-1));
    cube_template.push_back(octomath::Vector3( 1, 1,-1));
    cube_template.push_back(octomath::Vector3(-1, 1,-1));
    cube_template.push_back(octomath::Vector3( 1, 1,-1));
    cube_template.push_back(octomath::Vector3( 1, 1, 1));

    cube_template.push_back(octomath::Vector3(-1, 1,-1));
    cube_template.push_back(octomath::Vector3(-1,-1,-1));
    cube_template.push_back(octomath::Vector3( 1, 1, 1));
    cube_template.push_back(octomath::Vector3(-1, 1, 1));
    cube_template.push_back(octomath::Vector3( 1,-1,-1));
    cube_template.push_back(octomath::Vector3( 1,-1, 1));

    cube_template.push_back(octomath::Vector3(-1, 1, 1));
    cube_template.push_back(octomath::Vector3(-1,-1, 1));
    cube_template.push_back(octomath::Vector3( 1,-1, 1));
    cube_template.push_back(octomath::Vector3(-1,-1, 1));
    cube_template.push_back(octomath::Vector3(-1,-1,-1));
    cube_template.push_back(octomath::Vector3(-1,-1, 1));

    cube_template.push_back(octomath::Vector3( 1, 1, 1));
    cube_template.push_back(octomath::Vector3( 1,-1, 1));
    cube_template.push_back(octomath::Vector3( 1,-1,-1));
    cube_template.push_back(octomath::Vector3(-1,-1,-1));
    cube_template.push_back(octomath::Vector3(-1, 1,-1));
    cube_template.push_back(octomath::Vector3(-1, 1, 1));
  }

  unsigned int OcTreeDrawer::generateCube(const octomap::OcTreeVolume& v,
                                          const std::vector<octomath::Vector3>& cube_template,
                                          const unsigned int& current_array_idx,
                                          GLfloat*** glArray) {

    // epsilon to be substracted from cube size so that neighboring planes don't overlap
    // seems to introduce strange artifacts nevertheless...
    double eps = 1e-5;
    
    octomath::Vector3 p;

    double half_cube_size = GLfloat(v.second /2.0 -eps);
    unsigned int i = current_array_idx;
    // Cube surfaces are in gl_array in order: back, front, top, down, left, right.
    // Arrays are filled in parallel (increasing i for all at once)
    // One color array for all surfaces is filled when requested

    p = v.first + cube_template[0] * half_cube_size;
    (*glArray)[0][i]   = p.x();
    (*glArray)[0][i+1] = p.y();
    (*glArray)[0][i+2] = p.z();

    p = v.first + cube_template[1] * half_cube_size;
    (*glArray)[1][i]   = p.x();
    (*glArray)[1][i+1] = p.y();
    (*glArray)[1][i+2] = p.z();

    p = v.first + cube_template[2] * half_cube_size;
    (*glArray)[2][i]   = p.x();
    (*glArray)[2][i+1] = p.y();
    (*glArray)[2][i+2] = p.z();

    p = v.first + cube_template[3] * half_cube_size;
    (*glArray)[3][i]   = p.x(); 
    (*glArray)[3][i+1] = p.y(); 
    (*glArray)[3][i+2] = p.z(); 

    p = v.first + cube_template[4] * half_cube_size;
    (*glArray)[4][i]   = p.x(); 
    (*glArray)[4][i+1] = p.y(); 
    (*glArray)[4][i+2] = p.z(); 

    p = v.first + cube_template[5] * half_cube_size;
    (*glArray)[5][i]   = p.x(); 
    (*glArray)[5][i+1] = p.y(); 
    (*glArray)[5][i+2] = p.z(); 
    i+= 3;  //-------------------

    p = v.first + cube_template[6] * half_cube_size;
    (*glArray)[0][i]   = p.x();
    (*glArray)[0][i+1] = p.y();
    (*glArray)[0][i+2] = p.z();

    p = v.first + cube_template[7] * half_cube_size;
    (*glArray)[1][i]   = p.x();
    (*glArray)[1][i+1] = p.y();
    (*glArray)[1][i+2] = p.z();

    p = v.first + cube_template[8] * half_cube_size;
    (*glArray)[2][i]   = p.x();
    (*glArray)[2][i+1] = p.y();
    (*glArray)[2][i+2] = p.z();

    p = v.first + cube_template[9] * half_cube_size;
    (*glArray)[3][i]   = p.x();
    (*glArray)[3][i+1] = p.y();
    (*glArray)[3][i+2] = p.z();

    p = v.first + cube_template[10] * half_cube_size;
    (*glArray)[4][i]   = p.x();
    (*glArray)[4][i+1] = p.y();
    (*glArray)[4][i+2] = p.z();

    p = v.first + cube_template[11] * half_cube_size;
    (*glArray)[5][i]   = p.x();
    (*glArray)[5][i+1] = p.y();
    (*glArray)[5][i+2] = p.z();
    i+= 3;  //-------------------

    p = v.first + cube_template[12] * half_cube_size;
    (*glArray)[0][i]   = p.x();
    (*glArray)[0][i+1] = p.y();
    (*glArray)[0][i+2] = p.z();

    p = v.first + cube_template[13] * half_cube_size;
    (*glArray)[1][i]   = p.x();
    (*glArray)[1][i+1] = p.y();
    (*glArray)[1][i+2] = p.z();

    p = v.first + cube_template[14] * half_cube_size;
    (*glArray)[2][i]   = p.x();
    (*glArray)[2][i+1] = p.y();
    (*glArray)[2][i+2] = p.z();

    p = v.first + cube_template[15] * half_cube_size;
    (*glArray)[3][i]   = p.x();
    (*glArray)[3][i+1] = p.y();
    (*glArray)[3][i+2] = p.z();

    p = v.first + cube_template[16] * half_cube_size;
    (*glArray)[4][i]   = p.x();
    (*glArray)[4][i+1] = p.y();
    (*glArray)[4][i+2] = p.z();

    p = v.first + cube_template[17] * half_cube_size;
    (*glArray)[5][i]   = p.x();
    (*glArray)[5][i+1] = p.y();
    (*glArray)[5][i+2] = p.z();
    i+= 3;  //-------------------

    p = v.first + cube_template[18] * half_cube_size;
    (*glArray)[0][i]   = p.x();
    (*glArray)[0][i+1] = p.y();
    (*glArray)[0][i+2] = p.z();

    p = v.first + cube_template[19] * half_cube_size;
    (*glArray)[1][i]   = p.x();
    (*glArray)[1][i+1] = p.y();
    (*glArray)[1][i+2] = p.z();

    p = v.first + cube_template[20] * half_cube_size;
    (*glArray)[2][i]   = p.x();
    (*glArray)[2][i+1] = p.y();
    (*glArray)[2][i+2] = p.z();

    p = v.first + cube_template[21] * half_cube_size;
    (*glArray)[3][i]   = p.x();
    (*glArray)[3][i+1] = p.y();
    (*glArray)[3][i+2] = p.z();

    p = v.first + cube_template[22] * half_cube_size;
    (*glArray)[4][i]   = p.x();
    (*glArray)[4][i+1] = p.y();
    (*glArray)[4][i+2] = p.z();

    p = v.first + cube_template[23] * half_cube_size;
    (*glArray)[5][i]   = p.x();
    (*glArray)[5][i+1] = p.y();
    (*glArray)[5][i+2] = p.z();
    i += 3;  //-------------------
    
    return i; // updated array idx
  }


  unsigned int OcTreeDrawer::setCubeColorHeightmap(const octomap::OcTreeVolume& v,
                                          const unsigned int& current_array_idx,
                                          GLfloat** glColorArray) {

    if (glColorArray == NULL) return current_array_idx;

    unsigned int colorIdx = current_array_idx;
    // color for all 4 vertices (same height)
    for (int k = 0; k < 4; ++k) {
      //if (m_colorMode == CM_GRAY_HEIGHT)
       // SceneObject::heightMapGray(v.first.z(), *glColorArray + colorIdx);  // sets r,g,b
      //else
      //  SceneObject::heightMapColor(v.first.z(), *glColorArray + colorIdx);   // sets r,g,b
      // set Alpha value:
      (*glColorArray)[colorIdx + 3] = m_alphaOccupied;
      colorIdx += 4;
    }  
    return colorIdx;
  }

  unsigned int OcTreeDrawer::setCubeColorRGBA(const unsigned char& r,
                                              const unsigned char& g,
                                              const unsigned char& b,
                                              const unsigned char& a,
                                              const unsigned int& current_array_idx,
                                              GLfloat** glColorArray) {

    if (glColorArray == NULL) return current_array_idx;
    unsigned int colorIdx = current_array_idx;
    // set color for next 4 vertices (=one quad)
    for (int k = 0; k < 4; ++k) {
      (*glColorArray)[colorIdx    ] = (double) r/255.;
      (*glColorArray)[colorIdx + 1] = (double) g/255.;
      (*glColorArray)[colorIdx + 2] = (double) b/255.;
      (*glColorArray)[colorIdx + 3] = (double) a/255.;
      colorIdx += 4;
    }  
    return colorIdx;
  }




void OcTreeDrawer::setOcTreeSelection(const std::list<octomap::OcTreeVolume>& selectedVoxels)
{
  generateCubes(selectedVoxels, &m_selectionArray, m_selectionSize);

}

void OcTreeDrawer::clearOcTreeSelection()
{
  clearCubes(&m_selectionArray, m_selectionSize);
}

void OcTreeDrawer::generateCubes(const std::list<octomap::OcTreeVolume>& voxels, GLfloat*** glArray,
    unsigned int& glArraySize, GLfloat** glColorArray)
{

  // clear arrays first if needed:
  clearCubes(glArray, glArraySize, glColorArray);

  // now, allocate arrays:
  glArraySize = voxels.size() * 4 * 3;
  *glArray = new GLfloat*[6];

  for (unsigned i = 0; i < 6; ++i) {
    (*glArray)[i] = new GLfloat[glArraySize];
  }

  if (glColorArray != NULL
  )
    *glColorArray = new GLfloat[glArraySize * 4 * 4];

    // epsilon to be substracted from cube size so that neighboring planes don't overlap
    // seems to introduce strange artifacts nevertheless...
double  eps = 1e-5;

  // generate the cubes, 6 quads each
  // min and max-values are computed on-the-fly
  unsigned int i = 0;
  unsigned int colorIdx = 0;
  double x, y, z;

  for (std::list<octomap::OcTreeVolume>::const_iterator it = voxels.begin(); it != voxels.end(); it++) {

    double half_cube_size = GLfloat(it->second / 2.0 - eps);

    x = it->first.x();
    y = it->first.y();
    z = it->first.z();

    // Cube surfaces are in gl_array in order: back, front, top, down, left, right.
    // Arrays are filled in parrallel (increasing i for all at once)
    // One color array for all surfaces is filled when requested

    (*glArray)[0][i] = x + half_cube_size;
    (*glArray)[0][i + 1] = y + half_cube_size;
    (*glArray)[0][i + 2] = z - half_cube_size;

    (*glArray)[1][i] = x + half_cube_size;
    (*glArray)[1][i + 1] = y - half_cube_size;
    (*glArray)[1][i + 2] = z - half_cube_size;

    (*glArray)[2][i] = x + half_cube_size;
    (*glArray)[2][i + 1] = y + half_cube_size;
    (*glArray)[2][i + 2] = z - half_cube_size;

    (*glArray)[3][i] = x - half_cube_size;
    (*glArray)[3][i + 1] = y + half_cube_size;
    (*glArray)[3][i + 2] = z - half_cube_size;

    (*glArray)[4][i] = x + half_cube_size;
    (*glArray)[4][i + 1] = y + half_cube_size;
    (*glArray)[4][i + 2] = z - half_cube_size;

    (*glArray)[5][i] = x + half_cube_size;
    (*glArray)[5][i + 1] = y + half_cube_size;
    (*glArray)[5][i + 2] = z + half_cube_size;
    i += 3;

    (*glArray)[0][i] = x - half_cube_size;
    (*glArray)[0][i + 1] = y + half_cube_size;
    (*glArray)[0][i + 2] = z - half_cube_size;

    (*glArray)[1][i] = x - half_cube_size;
    (*glArray)[1][i + 1] = y - half_cube_size;
    (*glArray)[1][i + 2] = z - half_cube_size;

    (*glArray)[2][i] = x + half_cube_size;
    (*glArray)[2][i + 1] = y + half_cube_size;
    (*glArray)[2][i + 2] = z + half_cube_size;

    (*glArray)[3][i] = x - half_cube_size;
    (*glArray)[3][i + 1] = y + half_cube_size;
    (*glArray)[3][i + 2] = z + half_cube_size;

    (*glArray)[4][i] = x + half_cube_size;
    (*glArray)[4][i + 1] = y - half_cube_size;
    (*glArray)[4][i + 2] = z - half_cube_size;

    (*glArray)[5][i] = x + half_cube_size;
    (*glArray)[5][i + 1] = y - half_cube_size;
    (*glArray)[5][i + 2] = z + half_cube_size;
    i += 3;

    (*glArray)[0][i] = x - half_cube_size;
    (*glArray)[0][i + 1] = y + half_cube_size;
    (*glArray)[0][i + 2] = z + half_cube_size;

    (*glArray)[1][i] = x - half_cube_size;
    (*glArray)[1][i + 1] = y - half_cube_size;
    (*glArray)[1][i + 2] = z + half_cube_size;

    (*glArray)[2][i] = x + half_cube_size;
    (*glArray)[2][i + 1] = y - half_cube_size;
    (*glArray)[2][i + 2] = z + half_cube_size;

    (*glArray)[3][i] = x - half_cube_size;
    (*glArray)[3][i + 1] = y - half_cube_size;
    (*glArray)[3][i + 2] = z + half_cube_size;

    (*glArray)[4][i] = x - half_cube_size;
    (*glArray)[4][i + 1] = y - half_cube_size;
    (*glArray)[4][i + 2] = z - half_cube_size;

    (*glArray)[5][i] = x - half_cube_size;
    (*glArray)[5][i + 1] = y - half_cube_size;
    (*glArray)[5][i + 2] = z + half_cube_size;
    i += 3;

    (*glArray)[0][i] = x + half_cube_size;
    (*glArray)[0][i + 1] = y + half_cube_size;
    (*glArray)[0][i + 2] = z + half_cube_size;

    (*glArray)[1][i] = x + half_cube_size;
    (*glArray)[1][i + 1] = y - half_cube_size;
    (*glArray)[1][i + 2] = z + half_cube_size;

    (*glArray)[2][i] = x + half_cube_size;
    (*glArray)[2][i + 1] = y - half_cube_size;
    (*glArray)[2][i + 2] = z - half_cube_size;

    (*glArray)[3][i] = x - half_cube_size;
    (*glArray)[3][i + 1] = y - half_cube_size;
    (*glArray)[3][i + 2] = z - half_cube_size;

    (*glArray)[4][i] = x - half_cube_size;
    (*glArray)[4][i + 1] = y + half_cube_size;
    (*glArray)[4][i + 2] = z - half_cube_size;

    (*glArray)[5][i] = x - half_cube_size;
    (*glArray)[5][i + 1] = y + half_cube_size;
    (*glArray)[5][i + 2] = z + half_cube_size;
    i += 3;

    if (glColorArray != NULL) {
      // color for 4 vertices (same height)
      for (int k = 0; k < 4; ++k) {

        //          SceneObject::heightMapColor(z, *glColorArray + colorIdx);
        double z_norm = (z - m_zMin) / (m_zMax - m_zMin);
        float * jetC = bot_color_util_jet(z_norm);
        memcpy(*glColorArray + colorIdx, jetC, 3 * sizeof(float));
        // set Alpha value:
        (*glColorArray)[colorIdx + 3] = m_alphaOccupied;
        colorIdx += 4;
      }
    }

  }
}

void OcTreeDrawer::clearCubes(GLfloat*** glArray, unsigned int& glArraySize, GLfloat** glColorArray)
{
  if (glArraySize != 0) {
    for (unsigned i = 0; i < 6; ++i) {
      delete[] (*glArray)[i];
    }
    delete[] *glArray;
    *glArray = NULL;
    glArraySize = 0;
  }

  if (glColorArray != NULL && *glColorArray != NULL) {
    delete[] *glColorArray;
    *glColorArray = NULL;
  }
}

void OcTreeDrawer::initOctreeGridVis()
{

  if (m_octree_grid_vis_initialized)
    return;

  clearOcTreeStructure();
  // allocate arrays for octree grid visualization
  octree_grid_vertex_size = m_grid_voxels.size() * 12 * 2 * 3;
  octree_grid_vertex_array = new GLfloat[octree_grid_vertex_size];

  // generate the cubes, 12 lines each

  std::list<octomap::OcTreeVolume>::iterator it_rec;
  unsigned int i = 0;
  double x, y, z;
  for (it_rec = m_grid_voxels.begin(); it_rec != m_grid_voxels.end(); it_rec++) {

    x = it_rec->first.x();
    y = it_rec->first.y();
    z = it_rec->first.z();

    double half_voxel_size = it_rec->second / 2.0;

    // ----
    octree_grid_vertex_array[i] = x + half_voxel_size;
    octree_grid_vertex_array[i + 1] = y + half_voxel_size;
    octree_grid_vertex_array[i + 2] = z - half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x - half_voxel_size;
    octree_grid_vertex_array[i + 1] = y + half_voxel_size;
    octree_grid_vertex_array[i + 2] = z - half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x - half_voxel_size;
    octree_grid_vertex_array[i + 1] = y + half_voxel_size;
    octree_grid_vertex_array[i + 2] = z - half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x - half_voxel_size;
    octree_grid_vertex_array[i + 1] = y + half_voxel_size;
    octree_grid_vertex_array[i + 2] = z + half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x - half_voxel_size;
    octree_grid_vertex_array[i + 1] = y + half_voxel_size;
    octree_grid_vertex_array[i + 2] = z + half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x + half_voxel_size;
    octree_grid_vertex_array[i + 1] = y + half_voxel_size;
    octree_grid_vertex_array[i + 2] = z + half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x + half_voxel_size;
    octree_grid_vertex_array[i + 1] = y + half_voxel_size;
    octree_grid_vertex_array[i + 2] = z + half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x + half_voxel_size;
    octree_grid_vertex_array[i + 1] = y + half_voxel_size;
    octree_grid_vertex_array[i + 2] = z - half_voxel_size;
    i += 3;
    // ----
    octree_grid_vertex_array[i] = x + half_voxel_size;
    octree_grid_vertex_array[i + 1] = y - half_voxel_size;
    octree_grid_vertex_array[i + 2] = z + half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x - half_voxel_size;
    octree_grid_vertex_array[i + 1] = y - half_voxel_size;
    octree_grid_vertex_array[i + 2] = z + half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x - half_voxel_size;
    octree_grid_vertex_array[i + 1] = y - half_voxel_size;
    octree_grid_vertex_array[i + 2] = z + half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x - half_voxel_size;
    octree_grid_vertex_array[i + 1] = y - half_voxel_size;
    octree_grid_vertex_array[i + 2] = z - half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x - half_voxel_size;
    octree_grid_vertex_array[i + 1] = y - half_voxel_size;
    octree_grid_vertex_array[i + 2] = z - half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x + half_voxel_size;
    octree_grid_vertex_array[i + 1] = y - half_voxel_size;
    octree_grid_vertex_array[i + 2] = z - half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x + half_voxel_size;
    octree_grid_vertex_array[i + 1] = y - half_voxel_size;
    octree_grid_vertex_array[i + 2] = z - half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x + half_voxel_size;
    octree_grid_vertex_array[i + 1] = y - half_voxel_size;
    octree_grid_vertex_array[i + 2] = z + half_voxel_size;
    i += 3;
    // ----
    octree_grid_vertex_array[i] = x + half_voxel_size;
    octree_grid_vertex_array[i + 1] = y + half_voxel_size;
    octree_grid_vertex_array[i + 2] = z - half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x + half_voxel_size;
    octree_grid_vertex_array[i + 1] = y - half_voxel_size;
    octree_grid_vertex_array[i + 2] = z - half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x - half_voxel_size;
    octree_grid_vertex_array[i + 1] = y + half_voxel_size;
    octree_grid_vertex_array[i + 2] = z - half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x - half_voxel_size;
    octree_grid_vertex_array[i + 1] = y - half_voxel_size;
    octree_grid_vertex_array[i + 2] = z - half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x - half_voxel_size;
    octree_grid_vertex_array[i + 1] = y + half_voxel_size;
    octree_grid_vertex_array[i + 2] = z + half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x - half_voxel_size;
    octree_grid_vertex_array[i + 1] = y - half_voxel_size;
    octree_grid_vertex_array[i + 2] = z + half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x + half_voxel_size;
    octree_grid_vertex_array[i + 1] = y + half_voxel_size;
    octree_grid_vertex_array[i + 2] = z + half_voxel_size;
    i += 3;
    octree_grid_vertex_array[i] = x + half_voxel_size;
    octree_grid_vertex_array[i + 1] = y - half_voxel_size;
    octree_grid_vertex_array[i + 2] = z + half_voxel_size;
    i += 3;
    // ----
  }

  m_octree_grid_vis_initialized = true;
}

void OcTreeDrawer::clearOcTreeStructure()
{
  if (octree_grid_vertex_size != 0) {
    delete[] octree_grid_vertex_array;
    octree_grid_vertex_size = 0;
  }

  m_octree_grid_vis_initialized = false;
}

void OcTreeDrawer::clear()
{
  //clearOcTree();
  clearCubes(&m_occupiedArray, m_occupiedSize, &m_occupiedColorArray);
  clearCubes(&m_occupiedThresArray, m_occupiedThresSize, &m_occupiedThresColorArray);
  clearCubes(&m_freeArray, m_freeSize);
  clearCubes(&m_freeThresArray, m_freeThresSize);
  clearCubes(&m_selectionArray, m_selectionSize);

  clearOcTreeStructure();
}

void OcTreeDrawer::draw() const
{
  glPushAttrib(GL_ALL_ATTRIB_BITS);
  glPushMatrix();
  glMultMatrixf(m_ocTreeTransform);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  //        glEnable(GL_LIGHTING);

  glEnableClientState(GL_VERTEX_ARRAY);

  if (m_drawOccupied)
    drawOccupiedVoxels();
  if (m_drawFree)
    drawFreeVoxels();
  if (m_drawOcTreeGrid)
    drawOctreeGrid();
  if (m_drawSelection)
    drawSelection();

  glDisableClientState(GL_VERTEX_ARRAY);

  glPopMatrix();
  glPopAttrib();

}

void OcTreeDrawer::drawOccupiedVoxels() const
{

  // draw binary occupied cells
  if (m_occupiedThresSize != 0) {
    glColor4f(0.0, 0.0, 0.0, m_alphaOccupied);
    drawCubes(m_occupiedThresArray, m_occupiedThresSize, m_occupiedThresColorArray);
  }

  // draw delta occupied cells
  if (m_occupiedSize != 0) {
    glColor4f(0.2, 0.7, 1.0, m_alphaOccupied);
    drawCubes(m_occupiedArray, m_occupiedSize, m_occupiedColorArray);
  }

}

void OcTreeDrawer::drawFreeVoxels() const
{

  // draw binary freespace cells
  if (m_freeThresSize != 0) {
    glColor4f(0.0, 1.0, 0., 0.3);
    drawCubes(m_freeThresArray, m_freeThresSize);
  }

  // draw delta freespace cells
  if (m_freeSize != 0) {
    glColor4f(0.5, 1.0, 0.1, 0.3);
    drawCubes(m_freeArray, m_freeSize);
  }
}

void OcTreeDrawer::drawSelection() const
{
  if (m_selectionSize != 0) {
    glColor4f(1.0, 0.0, 0.0, 0.5);
    drawCubes(m_selectionArray, m_selectionSize);
  }
}

void OcTreeDrawer::drawCubes(GLfloat** cubeArray, unsigned int cubeArraySize, GLfloat* cubeColorArray) const
    {
  if (cubeArraySize == 0 || cubeArray == NULL) {
    std::cerr << "Warning: GLfloat array to draw cubes appears to be empty, nothing drawn.\n";
    return;
  }

  // save current color
  GLfloat* curcol = new GLfloat[4];
  glGetFloatv(GL_CURRENT_COLOR, curcol);

  // enable color pointer when heightColorMode is enabled:

  if (m_heightColorMode && cubeColorArray != NULL) {
    glEnableClientState(GL_COLOR_ARRAY);
    glColorPointer(4, GL_FLOAT, 0, cubeColorArray);
  }
  else{
    glColor4f(.1,.1,.1,.5);
  }

  // top surfaces:
  glNormal3f(0.0f, 1.0f, 0.0f);
  glVertexPointer(3, GL_FLOAT, 0, cubeArray[0]);
  glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);
  // bottom surfaces:
  glNormal3f(0.0f, -1.0f, 0.0f);
  glVertexPointer(3, GL_FLOAT, 0, cubeArray[1]);
  glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);
  // right surfaces:
  glNormal3f(1.0f, 0.0f, 0.0f);
  glVertexPointer(3, GL_FLOAT, 0, cubeArray[2]);
  glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);
  // left surfaces:
  glNormal3f(-1.0f, 0.0f, 0.0f);
  glVertexPointer(3, GL_FLOAT, 0, cubeArray[3]);
  glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);
  // back surfaces:
  glNormal3f(0.0f, 0.0f, -1.0f);
  glVertexPointer(3, GL_FLOAT, 0, cubeArray[4]);
  glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);
  // front surfaces:
  glNormal3f(0.0f, 0.0f, 1.0f);
  glVertexPointer(3, GL_FLOAT, 0, cubeArray[5]);
  glDrawArrays(GL_QUADS, 0, cubeArraySize / 3);

  if (m_heightColorMode && cubeColorArray != NULL) {
    glDisableClientState(GL_COLOR_ARRAY);
  }

  // reset color
  glColor4fv(curcol);
  delete[] curcol;
}

void OcTreeDrawer::drawOctreeGrid() const
{
  if (!m_octree_grid_vis_initialized)
    return;

  if (octree_grid_vertex_size == 0)
    return;

  glDisable(GL_LIGHTING);
  glEnable(GL_LINE_SMOOTH);

  glLineWidth(1.);
  glVertexPointer(3, GL_FLOAT, 0, octree_grid_vertex_array);
  glColor3f(0.0, 0.0, 0.0);
  glDrawArrays(GL_LINES, 0, octree_grid_vertex_size / 3);

  glDisable(GL_LINE_SMOOTH);
  glEnable(GL_LIGHTING);
}

void OcTreeDrawer::enableOcTree(bool enabled)
{
  m_drawOcTreeGrid = enabled;
  if (m_drawOcTreeGrid && !m_octree_grid_vis_initialized) {
    initOctreeGridVis();
  }
}
}


///////////////// End of OcTreeDrawer ///////////////////


namespace octomap {

  ColorOcTreeDrawer::ColorOcTreeDrawer() 
    : OcTreeDrawer() {
  }

  ColorOcTreeDrawer::~ColorOcTreeDrawer() {
  }

  void ColorOcTreeDrawer::setOcTree(const AbstractOcTree& tree_pnt,
                                    const octomap::pose6d& origin_,
                                    int map_id_) {

    const ColorOcTree& tree = ((const ColorOcTree&) tree_pnt);

    this->map_id = map_id_;

    // save origin used during cube generation
    this->initial_origin = octomap::pose6d(octomap::point3d(0,0,0), origin_.rot());
    // origin is in global coords
    this->origin = origin_;
    
    // maximum size to prevent crashes on large maps: (should be checked in a better way than a constant)
    bool showAll = (tree.size() < 5 * 1e6);
    bool uses_origin = ( (origin_.rot().x() != 0.) && (origin_.rot().y() != 0.)
                         && (origin_.rot().z() != 0.) && (origin_.rot().u() != 1.) );

    // walk the tree one to find the number of nodes in each category
    // (this is used to set up the OpenGL arrays)
    // TODO: this step may be left out, if we maintained the GLArrays in std::vectors instead...
    unsigned int cnt_occupied(0), cnt_occupied_thres(0), cnt_free(0), cnt_free_thres(0);
    for(ColorOcTree::tree_iterator it = tree.begin_tree(this->m_max_tree_depth),
          end=tree.end_tree(); it!= end; ++it) {
      if (it.isLeaf()) { 
        if (tree.isNodeOccupied(*it)){ // occupied voxels
          if (tree.isNodeAtThreshold(*it)) ++cnt_occupied_thres;
          else                               ++cnt_occupied;
        }
        else if (showAll) { // freespace voxels
          if (tree.isNodeAtThreshold(*it)) ++cnt_free_thres;
          else                               ++cnt_free;
        }
      }        
    }    
    // setup GL arrays for cube quads and cube colors
    initGLArrays(cnt_occupied      , m_occupiedSize     , &m_occupiedArray     , &m_occupiedColorArray);
    initGLArrays(cnt_occupied_thres, m_occupiedThresSize, &m_occupiedThresArray, &m_occupiedThresColorArray);
    initGLArrays(cnt_free          , m_freeSize         , &m_freeArray, NULL);
    initGLArrays(cnt_free_thres    , m_freeThresSize    , &m_freeThresArray, NULL);

    std::vector<octomath::Vector3> cube_template;
    initCubeTemplate(origin, cube_template);

    unsigned int idx_occupied(0), idx_occupied_thres(0), idx_free(0), idx_free_thres(0);
    unsigned int color_idx_occupied(0), color_idx_occupied_thres(0);

    m_grid_voxels.clear();
    OcTreeVolume voxel; // current voxel, possibly transformed 
    for(ColorOcTree::tree_iterator it = tree.begin_tree(this->m_max_tree_depth),
          end=tree.end_tree(); it!= end; ++it) {

      if (it.isLeaf()) { // voxels for leaf nodes
        if (uses_origin) 
          voxel = OcTreeVolume(origin.rot().rotate(it.getCoordinate()), it.getSize());
        else 
          voxel = OcTreeVolume(it.getCoordinate(), it.getSize());
        
        if (tree.isNodeOccupied(*it)){ // occupied voxels
          if (tree.isNodeAtThreshold(*it)) {
            idx_occupied_thres = generateCube(voxel, cube_template, idx_occupied_thres, &m_occupiedThresArray);
            color_idx_occupied_thres =  setCubeColorRGBA(it->getColor().r, it->getColor().g, it->getColor().b, 
                                                         (unsigned char) (it->getOccupancy() * 255.),
                                                         color_idx_occupied_thres, &m_occupiedThresColorArray);
          }
          else {
            idx_occupied = generateCube(voxel, cube_template, idx_occupied, &m_occupiedArray);
            color_idx_occupied = setCubeColorRGBA(it->getColor().r, it->getColor().g, it->getColor().b, 
                                                  (unsigned char)(it->getOccupancy() * 255.),
                                                  color_idx_occupied, &m_occupiedColorArray);
          }
        }
        else if (showAll) { // freespace voxels
          if (tree.isNodeAtThreshold(*it)) {
            idx_free_thres = generateCube(voxel, cube_template, idx_free_thres, &m_freeThresArray);
          }
          else {
            idx_free = generateCube(voxel, cube_template, idx_free, &m_freeArray);
          }
        }

        // grid structure voxel
        if (showAll) m_grid_voxels.push_back(voxel);        
      }
      
      else { // inner node voxels (for grid structure only)
        if (showAll) {
          if (uses_origin)
            voxel = OcTreeVolume(origin.rot().rotate(it.getCoordinate()), it.getSize());
          else
            voxel = OcTreeVolume(it.getCoordinate(), it.getSize());
          m_grid_voxels.push_back(voxel);
        }
      }      
    } // end for all voxels

    m_octree_grid_vis_initialized = false;

    if(m_drawOcTreeGrid)
      initOctreeGridVis();    
  }

} // end namespace





using namespace octomap;



vtkStandardNewMacro(vtkOctomap);


class vtkOctomap::vtkInternal {
public:
  vtkInternal()
    {
      this->msg.length = 0;
    }

  octomap_raw_t msg;
  OcTree * ocTree;
  OcTreeDrawer * octd;

    std::map<int, OcTreeRecord> m_octrees;
    //ViewerWidget* m_glwidget;

  std::vector<vtkSmartPointer<vtkActor> > Actors;
};

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

  this->Internal->ocTree = NULL;
  this->Internal->octd = new OcTreeDrawer();

//  this->Internal->octd->m_heightColorMode = true;
  this->Internal->octd->m_zMin = -1.0;
  this->Internal->octd->m_zMax = 12.0;
}

//----------------------------------------------------------------------------
vtkOctomap::~vtkOctomap()
{
  delete this->Internal;
}

//----------------------------------------------------------------------------

void vtkOctomap::SetHeightColorMode(bool heightColorMode){
  this->Internal->octd->m_heightColorMode = heightColorMode;
}

void vtkOctomap::SetAlphaOccupied(double alphaOccupied){
  this->Internal->octd->setAlphaOccupied(alphaOccupied);

  // Currently necessary to force a recoloring
  //this->Internal->octd->setOcTree(*this->Internal->ocTree, this->Internal->octd->m_zMin,
  //    this->Internal->octd->m_zMax); //regenerate cubes etc..
}

//----------------------------------------------------------------------------


void vtkOctomap::UpdateOctomapData(const char* messageData)
{

  int status = octomap_raw_t_decode (messageData, 0, 1e9, &this->Internal->msg);

  if (!status)
  {
    this->Internal->msg.length = 0;
  }else{

    if (this->Internal->ocTree != NULL)
      delete this->Internal->ocTree;
    this->Internal->octd->clear();

    // set transform
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        this->Internal->octd->m_ocTreeTransform[j*4+i] = this->Internal->msg.transform[i][j];
      }
    }

    std::stringstream datastream;
    datastream.write((const char*) this->Internal->msg.data, this->Internal->msg.length);

    if (1==0){
    this->Internal->ocTree = new octomap::OcTree(1); //resolution will be set by data from message
    this->Internal->ocTree->readBinary(datastream);
    }else{
  std::string m_filename = "/home/mfallon/Dropbox/shared/2015-11-octomap/octomap.bt";
//  std::string m_filename = "/home/mfallon/Dropbox/shared/2015-11-octomap/simple_color_tree.ot";

std::cout << "read octomap from file\n";
      this->Internal->ocTree = new octomap::OcTree(m_filename);

    }

    double minX, minY, minZ, maxX, maxY, maxZ;
    this->Internal->ocTree->getMetricMin(minX, minY, minZ);
    this->Internal->ocTree->getMetricMax(maxX, maxY, maxZ);
    //printf("\nmap bounds: [%.2f, %.2f, %.2f] - [%.2f, %.2f, %.2f]\n", minX, minY, minZ, maxX, maxY, maxZ);
    this->Internal->octd->setOcTree(*this->Internal->ocTree);
//    this->Internal->octd->setOcTree(*this->Internal->ocTree,  this->Internal->octd->m_zMin ,
//      this->Internal->octd->m_zMax);
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

    //printf("vtkOctomap\n");
    this->Internal->octd->enableOcTreeCells();

    if (this->Internal->octd->m_drawOccupied || this->Internal->octd->m_drawFree || this->Internal->octd->m_drawOcTreeGrid) {
      if (this->Internal->ocTree == NULL) {
        return -1;
      }
    //printf("calling draw\n");
    this->Internal->octd->draw();
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
