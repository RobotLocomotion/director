/*=========================================================================

Program:   Visualization Toolkit

Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
All rights reserved.
See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

This software is distributed WITHOUT ANY WARRANTY; without even
the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkLCMGLProp - An orthonormal frame representation
// .SECTION Description
// An orthonormal frame representation for use with the vtkFrameWidget

// .SECTION See Also
// vtkFrameWidget vtkFrameWidget


#ifndef __vtkLCMGLProp_h
#define __vtkLCMGLProp_h

#include "vtkProp.h"

#include <vtkDRCFiltersModule.h>

class VTKDRCFILTERS_EXPORT vtkLCMGLProp : public vtkProp
{
public:
  // Description:
  // Instantiate the class.
  static vtkLCMGLProp *New();

  // Description:
  // Standard methods for the class.
  vtkTypeMacro(vtkLCMGLProp,vtkProp);
  void PrintSelf(ostream& os, vtkIndent indent) VTKDRCFILTERS_OVERRIDE;

  void UpdateGLData(const char* data);

  virtual double *GetBounds() VTKDRCFILTERS_OVERRIDE;

  // Description:
  // Methods supporting, and required by, the rendering process.
  virtual void ReleaseGraphicsResources(vtkWindow*) VTKDRCFILTERS_OVERRIDE;
  virtual int RenderOpaqueGeometry(vtkViewport*) VTKDRCFILTERS_OVERRIDE;
  virtual int RenderOverlay(vtkViewport*) VTKDRCFILTERS_OVERRIDE;
  virtual int RenderTranslucentPolygonalGeometry(vtkViewport*) VTKDRCFILTERS_OVERRIDE;
  virtual int HasTranslucentPolygonalGeometry() VTKDRCFILTERS_OVERRIDE;

protected:
  vtkLCMGLProp();
  virtual ~vtkLCMGLProp() VTKDRCFILTERS_OVERRIDE;

private:

  class vtkInternal;
  vtkInternal* Internal;

  vtkLCMGLProp(const vtkLCMGLProp&) VTKDRCFILTERS_DELETE_FUNCTION;
  void operator=(const vtkLCMGLProp&) VTKDRCFILTERS_DELETE_FUNCTION;
};

#endif
