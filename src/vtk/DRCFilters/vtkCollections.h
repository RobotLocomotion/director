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
  void on_link_collection_data(const char* data);
  void on_points_collection_data(const char* data);
  void on_reset_collections_data(const char* data);
    
  // Description:
  // Methods supporting, and required by, the rendering process.
  virtual void ReleaseGraphicsResources(vtkWindow*);
  virtual int RenderOpaqueGeometry(vtkViewport*);
  virtual int RenderOverlay(vtkViewport*);
  virtual int RenderTranslucentPolygonalGeometry(vtkViewport*);
  virtual int HasTranslucentPolygonalGeometry();

  class vtkInternal;
  vtkInternal* Internal;

  // Getter and Setter
  int getCollectionsSize();
  int getCollectionsId(int mapIndex);
  std::string getCollectionsName(int mapIndex);
  int getCollectionsType(int mapIndex);
  bool getCollectionsShow(int mapIndex);

  void setEnabled(int id, bool show);
  void removeIdFromCollections(int id);

  void setRangeStart(double rangeStart);
  void setRangeEnd(double rangeEnd);
  void setAlphaPoints(double alphaPoints);

  void setFillScans(bool colorByTime);

  void setPointWidth(double pointWidth);
  void setPoseWidth(double poseWidth);
  void setColorPoses(bool colorPoses);

  void setColorByTime(bool colorByTime);

  void setElevationByTime(bool elevationByTime);
  void setElevationByCollection(bool elevationByCollection);
  void setMaxElevation(double maxElevation);

  void setShowToggle();

protected:
  vtkCollections();
  ~vtkCollections();

private:



  template <class MyCollection>
    void on_collection_data(const typename MyCollection::my_vs_collection_t *msg);

  void calculate_ranges(int64_t& range_start, int64_t& range_end);

  double time_elevation(int64_t id, double z, int collid);


  vtkCollections(const vtkCollections&);  //Not implemented
  void operator=(const vtkCollections&);  //Not implemented
};

#endif
