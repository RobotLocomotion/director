/*=========================================================================

Program:   Visualization Toolkit

Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
All rights reserved.
See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

This software is distributed WITHOUT ANY WARRANTY; without even
the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkOctomap - An orthonormal frame representation
// .SECTION Description
// An orthonormal frame representation for use with the vtkFrameWidget

// .SECTION See Also
// vtkFrameWidget vtkFrameWidget


#ifndef __vtkOctomap_h
#define __vtkOctomap_h

#include "vtkProp.h"

#include <vtkOpenGL.h>

#include <vtkDRCFiltersModule.h>

#include <QMap>
#include <QFileInfo>
#include <QtXml/QDomElement>

#include <octomap/AbstractOcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap/octomap_types.h>
#include <octomap/ColorOcTree.h>
#include <iostream>
#include <sstream>

namespace octomap {

  /**
  * Abstract base class for objects to be drawn in the ViewerWidget.
  *
  */
  class SceneObject {
  public:
    enum ColorMode {
      CM_FLAT,
      CM_PRINTOUT,
      CM_COLOR_HEIGHT,
      CM_GRAY_HEIGHT,
      CM_SEMANTIC
    };

  public:
    SceneObject();
    virtual ~SceneObject(){};

    /**
    * Actual draw function which will be called to visualize the object
    */
    virtual void draw() const = 0;

    /**
    * Clears the object's representation (will be called when it gets invalid)
    */
    virtual void clear(){};

  public:
    //! the color mode has to be set before calling OcTreDrawer::setMap()
    //! because the cubes are generated in OcTreDrawer::setMap() using the color information
    inline void setColorMode(ColorMode mode) { m_colorMode = mode; }
    inline void enablePrintoutMode(bool enabled = true) { if (enabled) m_colorMode = CM_PRINTOUT; else m_colorMode = CM_FLAT; }
    inline void enableHeightColorMode(bool enabled = true) { if (enabled) m_colorMode = CM_COLOR_HEIGHT; else m_colorMode = CM_FLAT; }
    inline void enableSemanticColoring(bool enabled = true) { if (enabled) m_colorMode = CM_SEMANTIC; else m_colorMode = CM_FLAT; }

  protected:
    /// writes rgb values which correspond to a rel. height in the map.
    /// (glArrayPos needs to have at least size 3!)
    void heightMapColor(double h, GLfloat* glArrayPos) const;
    void heightMapGray(double h, GLfloat* glArrayPos) const;
    double m_zMin;
    double m_zMax;
    ColorMode m_colorMode;
  };
}


namespace octomap {

class OcTreeDrawer: public SceneObject {
public:
  OcTreeDrawer();
  virtual ~OcTreeDrawer();
  void clear();
  void draw() const;
  /// sets a new OcTree that should be drawn by this drawer
  //void setOcTree(const octomap::OcTree &octree, double minDrawZ, double maxDrawZ);

  /// sets a new OcTree that should be drawn by this drawer
  void setOcTree(const AbstractOcTree& octree){
    octomap::pose6d o; // initialized to (0,0,0) , (0,0,0,1) by default
    setOcTree(octree, o, 0);
  }

  /// sets a new OcTree that should be drawn by this drawer
  /// origin specifies a global transformation that should be applied
  virtual void setOcTree(const AbstractOcTree& octree, const octomap::pose6d& origin, int map_id_);


  /// sets a new selection of the current OcTree to be drawn
  void setOcTreeSelection(const std::list<octomap::OcTreeVolume>& selectedPoints);

  /// clear the visualization of the OcTree selection
  void clearOcTreeSelection();

  /// sets alpha level for occupied cells
  void setAlphaOccupied(double alpha);

  void enableOcTree(bool enabled = true);
  void enableOcTreeCells(bool enabled = true)
  {
    m_drawOccupied = enabled;
  }
  ;
  void enableFreespace(bool enabled = true)
  {
    m_drawFree = enabled;
  }
  ;
  void enableSelection(bool enabled = true)
  {
    m_drawSelection = enabled;
  }
  ;
  void setMax_tree_depth(unsigned int max_tree_depth)
  {
    m_max_tree_depth = max_tree_depth;
  }
  ;

public:
  //void clearOcTree();
  void clearOcTreeStructure();

  void drawOctreeGrid() const;
  void drawOccupiedVoxels() const;
  void drawFreeVoxels() const;
  void drawSelection() const;
  void drawCubes(GLfloat** cubeArray, unsigned int cubeArraySize, GLfloat* cubeColorArray = NULL) const;

  //  void drawAxes() const;

  //! Initializes the OpenGL visualization for a list of OcTreeVolumes
  //! The array is cleared first, if needed
  void generateCubes(const std::list<octomap::OcTreeVolume>& points, GLfloat*** glArray, unsigned int& glArraySize,
      GLfloat** glColorArray = NULL);

  //! clear OpenGL visualization
  void clearCubes(GLfloat*** glArray, unsigned int& glArraySize, GLfloat** glColorArray = NULL);

    void initGLArrays(const unsigned int& num_cubes, unsigned int& glArraySize,
                       GLfloat*** glArray, GLfloat** glColorArray);
    //! setup cube template
    void initCubeTemplate(const octomath::Pose6D& origin,
                          std::vector<octomath::Vector3>& cube_template);
    //! add one cube to arrays
    unsigned int generateCube(const octomap::OcTreeVolume& v,
                              const std::vector<octomath::Vector3>& cube_template,
                              const unsigned int& current_array_idx,
                              GLfloat*** glArray);
    unsigned int setCubeColorHeightmap(const octomap::OcTreeVolume& v,
                                       const unsigned int& current_array_idx,
                                       GLfloat** glColorArray);
    unsigned int setCubeColorRGBA(const unsigned char& r, const unsigned char& g, 
                                  const unsigned char& b, const unsigned char& a,
                                  const unsigned int& current_array_idx,
                                  GLfloat** glColorArray);

  void initOctreeGridVis();

  //! OpenGL representation of Octree cells (cubes)

  GLfloat** m_occupiedThresArray;
  unsigned int m_occupiedThresSize;
  GLfloat** m_freeThresArray;
  unsigned int m_freeThresSize;
  GLfloat** m_occupiedArray;
  unsigned int m_occupiedSize;
  GLfloat** m_freeArray;
  unsigned int m_freeSize;
  GLfloat** m_selectionArray;
  unsigned int m_selectionSize;

  //! Color array for occupied cells (height)
  GLfloat* m_occupiedThresColorArray;
  GLfloat* m_occupiedColorArray;

  //! OpenGL representation of Octree (grid structure)
  // TODO: put in its own drawer object!
  GLfloat* octree_grid_vertex_array;
  unsigned int octree_grid_vertex_size;

  std::list<octomap::OcTreeVolume> m_grid_voxels;

  bool m_heightColorMode;
  double m_zMin, m_zMax;

  bool m_drawOccupied;
  bool m_drawOcTreeGrid;
  bool m_drawFree;
  bool m_drawSelection;
  bool m_octree_grid_vis_initialized;
    bool m_displayAxes;

  unsigned int m_max_tree_depth;
  double m_alphaOccupied;

  double minX, minY, minZ, maxX, maxY, maxZ;

  float m_ocTreeTransform[16];


    octomap::pose6d origin;
    octomap::pose6d initial_origin;

    int map_id;

};
} // namespace octomap


namespace octomap {

  class ColorOcTreeDrawer : public OcTreeDrawer {
  public:
    ColorOcTreeDrawer();
    virtual ~ColorOcTreeDrawer();

    virtual void setOcTree(const AbstractOcTree& tree_pnt, const pose6d& origin, int map_id_);

  protected:
    
    
  };


} // end namespace



namespace octomap {

  class OcTreeRecord {
  public:
    AbstractOcTree*  octree;
    OcTreeDrawer*    octree_drawer;
    unsigned int     id;
    pose6d           origin;
  };

} // namespace octomap

class VTKDRCFILTERS_EXPORT vtkOctomap : public vtkProp
{
public:
  // Description:
  // Instantiate the class.
  static vtkOctomap *New();

  // Description:
  // Standard methods for the class.
  vtkTypeMacro(vtkOctomap,vtkProp);
  void PrintSelf(ostream& os, vtkIndent indent);

  void UpdateOctomapData(const char* data);

  void SetHeightColorMode(bool heightColorMode);
  void SetAlphaOccupied(double alphaOccupied);

  // Description:
  // Methods supporting, and required by, the rendering process.
  virtual void ReleaseGraphicsResources(vtkWindow*);
  virtual int RenderOpaqueGeometry(vtkViewport*);
  virtual int RenderOverlay(vtkViewport*);
  virtual int RenderTranslucentPolygonalGeometry(vtkViewport*);
  virtual int HasTranslucentPolygonalGeometry();

  void enableHeightColorMode (bool enabled = true);

    // use this drawer id if loading files or none is specified in msg
    static const unsigned int DEFAULT_OCTREE_ID  = 0; 


    void addOctree(octomap::AbstractOcTree* tree, int id, octomap::pose6d origin);
    void addOctree(octomap::AbstractOcTree* tree, int id);
    bool getOctreeRecord(int id, octomap::OcTreeRecord*& otr);

protected:
  vtkOctomap();
  ~vtkOctomap();

private:



  bool m_heightColorMode;
    /// open "regular" file containing an octree
    void openOcTree(std::string filename);

    /// open binary format OcTree
    void openTree(std::string filename);

    void parseTree(std::string datastream_string);
    void parseOcTree(std::string datastream_string);

    void showOcTree();

  class vtkInternal;
  vtkInternal* Internal;

  vtkOctomap(const vtkOctomap&);  //Not implemented
  void operator=(const vtkOctomap&);  //Not implemented
};

#endif
