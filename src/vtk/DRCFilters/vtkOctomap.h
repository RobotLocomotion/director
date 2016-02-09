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

#include <octomap/AbstractOcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap/octomap_types.h>
#include <octomap/ColorOcTree.h>
#include <octovis/ColorOcTreeDrawer.h>
#include <octovis/OcTreeRecord.h>
#include <iostream>
#include <sstream>


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

  void setAlphaOccupied(double alphaOccupied);
  void changeTreeDepth(int depth);
  void setColorMode (int colorMode);
  void enableOctreeStructure (bool enabled);
  void enableOcTreeCells (bool enabled);
  void enableFreespace (bool enabled);

  // Description:
  // Methods supporting, and required by, the rendering process.
  virtual void ReleaseGraphicsResources(vtkWindow*);
  virtual int RenderOpaqueGeometry(vtkViewport*);
  virtual int RenderOverlay(vtkViewport*);
  virtual int RenderTranslucentPolygonalGeometry(vtkViewport*);
  virtual int HasTranslucentPolygonalGeometry();


    // use this drawer id if loading files or none is specified in msg
    static const unsigned int DEFAULT_OCTREE_ID  = 0; 


    void addOctree(octomap::AbstractOcTree* tree, int id, octomap::pose6d origin);
    void addOctree(octomap::AbstractOcTree* tree, int id);
    bool getOctreeRecord(int id, octomap::OcTreeRecord*& otr);

protected:
  vtkOctomap();
  ~vtkOctomap();

private:

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
