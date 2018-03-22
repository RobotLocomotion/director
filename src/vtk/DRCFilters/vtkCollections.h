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
  void PrintSelf(ostream& os, vtkIndent indent) VTKDRCFILTERS_OVERRIDE;

  void on_obj_collection_data(const char* data);
  void on_link_collection_data(const char* data);
  void on_points_collection_data(const char* data);
  void on_reset_collections_data(const char* data);
    
  // Description:
  // Methods supporting, and required by, the rendering process.
  virtual void ReleaseGraphicsResources(vtkWindow*) VTKDRCFILTERS_OVERRIDE;
  virtual int RenderOpaqueGeometry(vtkViewport*) VTKDRCFILTERS_OVERRIDE;
  virtual int RenderOverlay(vtkViewport*) VTKDRCFILTERS_OVERRIDE;
  virtual int RenderTranslucentPolygonalGeometry(vtkViewport*)
    VTKDRCFILTERS_OVERRIDE;
  virtual int HasTranslucentPolygonalGeometry() VTKDRCFILTERS_OVERRIDE;

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
  virtual ~vtkCollections() VTKDRCFILTERS_OVERRIDE;

private:



  template <class MyCollection>
    void on_collection_data(const typename MyCollection::my_vs_collection_t *msg);

  void calculate_ranges(int64_t& range_start, int64_t& range_end);

  double time_elevation(int64_t id, double z, int collid);


  vtkCollections(const vtkCollections&) VTKDRCFILTERS_DELETE_FUNCTION;
  void operator=(const vtkCollections&) VTKDRCFILTERS_DELETE_FUNCTION;
};

#endif
