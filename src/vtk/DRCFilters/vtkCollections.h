/*=========================================================================

Program:   Visualization Toolkit

Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
All rights reserved.
See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

This software is distributed WITHOUT ANY WARRANTY; without even
the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkCollections - An orthonormal frame representation
// .SECTION Description
// An orthonormal frame representation for use with the vtkFrameWidget

// .SECTION See Also
// vtkFrameWidget vtkFrameWidget


#ifndef __vtkCollections_h
#define __vtkCollections_h

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


class VTKDRCFILTERS_EXPORT vtkCollections : public vtkProp
{
public:
  // Description:
  // Instantiate the class.
  static vtkCollections *New();

  // Description:
  // Standard methods for the class.
  vtkTypeMacro(vtkCollections,vtkProp);
  void PrintSelf(ostream& os, vtkIndent indent);

  void on_obj_collection_data(const char* data);

  // Description:
  // Methods supporting, and required by, the rendering process.
  virtual void ReleaseGraphicsResources(vtkWindow*);
  virtual int RenderOpaqueGeometry(vtkViewport*);
  virtual int RenderOverlay(vtkViewport*);
  virtual int RenderTranslucentPolygonalGeometry(vtkViewport*);
  virtual int HasTranslucentPolygonalGeometry();


protected:
  vtkCollections();
  ~vtkCollections();

private:

  class vtkInternal;
  vtkInternal* Internal;

  vtkCollections(const vtkCollections&);  //Not implemented
  void operator=(const vtkCollections&);  //Not implemented
};

#endif
