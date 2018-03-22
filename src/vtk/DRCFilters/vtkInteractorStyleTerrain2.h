/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkInteractorStyleTerrain2.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkInteractorStyleTerrain2 - manipulate camera in scene with natural view up (e.g., terrain)
// .SECTION Description
// vtkInteractorStyleTerrain2 is used to manipulate a camera which is viewing
// a scene with a natural view up, e.g., terrain. The camera in such a
// scene is manipulated by specifying azimuth (angle around the view
// up vector) and elevation (the angle from the horizon).
//
// The mouse binding for this class is as follows. Left mouse click followed
// rotates the camera around the focal point using both elevation and azimuth
// invocations on the camera. Left mouse motion in the horizontal direction
// results in azimuth motion; left mouse motion in the vertical direction
// results in elevation motion. Therefore, diagonal motion results in a
// combination of azimuth and elevation. (If the shift key is held during
// motion, then only one of elevation or azimuth is invoked, depending on the
// whether the mouse motion is primarily horizontal or vertical.) Middle
// mouse button pans the camera across the scene (again the shift key has a
// similar effect on limiting the motion to the vertical or horizontal
// direction. The right mouse is used to dolly (e.g., a type of zoom) towards
// or away from the focal point.
//
// The class also supports some keypress events. The "r" key resets the
// camera.  The "e" key invokes the exit callback and by default exits the
// program. The "f" key sets a new camera focal point and flys towards that
// point. The "u" key invokes the user event. The "3" key toggles between 
// stereo and non-stero mode. The "l" key toggles on/off a latitude/longitude
// markers that can be used to estimate/control position.
// 

// .SECTION See Also
// vtkInteractorObserver vtkInteractorStyle vtk3DWidget

#ifndef __vtkInteractorStyleTerrain2_h
#define __vtkInteractorStyleTerrain2_h

#include "vtkInteractorStyle.h"

#include <vtkDRCFiltersModule.h>

class vtkPolyDataMapper;
class vtkSphereSource;
class vtkExtractEdges;

class VTKDRCFILTERS_EXPORT vtkInteractorStyleTerrain2 : public vtkInteractorStyle
{
public:
  // Description:
  // Instantiate the object.
  static vtkInteractorStyleTerrain2 *New();

  vtkTypeMacro(vtkInteractorStyleTerrain2,vtkInteractorStyle);
  void PrintSelf(ostream& os, vtkIndent indent) VTKDRCFILTERS_OVERRIDE;

  // Description:
  // Event bindings controlling the effects of pressing mouse buttons
  // or moving the mouse.
  virtual void OnMouseMove() VTKDRCFILTERS_OVERRIDE;
  virtual void OnLeftButtonDown() VTKDRCFILTERS_OVERRIDE;
  virtual void OnLeftButtonUp() VTKDRCFILTERS_OVERRIDE;
  virtual void OnMiddleButtonDown() VTKDRCFILTERS_OVERRIDE;
  virtual void OnMiddleButtonUp() VTKDRCFILTERS_OVERRIDE;
  virtual void OnRightButtonDown() VTKDRCFILTERS_OVERRIDE;
  virtual void OnRightButtonUp() VTKDRCFILTERS_OVERRIDE;

  // Description:
  // Override the "fly-to" (f keypress) for images.
  virtual void OnChar() VTKDRCFILTERS_OVERRIDE;

  // These methods for the different interactions in different modes
  // are overridden in subclasses to perform the correct motion.
  virtual void Rotate() VTKDRCFILTERS_OVERRIDE;
  virtual void Pan() VTKDRCFILTERS_OVERRIDE;
  virtual void Dolly() VTKDRCFILTERS_OVERRIDE;

  // Description:
  // Turn on/off the latitude/longitude lines.
  vtkSetMacro(LatLongLines,int);
  vtkGetMacro(LatLongLines,int);
  vtkBooleanMacro(LatLongLines,int);

protected:
  vtkInteractorStyleTerrain2();
  virtual ~vtkInteractorStyleTerrain2() VTKDRCFILTERS_OVERRIDE;

  // Internal helper attributes
  int LatLongLines;

  vtkSphereSource *LatLongSphere;
  vtkPolyDataMapper *LatLongMapper;
  vtkActor *LatLongActor;
  vtkExtractEdges *LatLongExtractEdges;

  void SelectRepresentation();
  void CreateLatLong();

  double MotionFactor;

private:
  vtkInteractorStyleTerrain2(const vtkInteractorStyleTerrain2&)
    VTKDRCFILTERS_DELETE_FUNCTION;
  void operator=(const vtkInteractorStyleTerrain2&)
    VTKDRCFILTERS_DELETE_FUNCTION;

};

#endif

