/*=========================================================================

Program:   Visualization Toolkit

Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
All rights reserved.
See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

This software is distributed WITHOUT ANY WARRANTY; without even
the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkFrameWidget - A widget for manipulating an orthonormal frame
// .SECTION Description
// This widget allows the translation and rotation of an orthonormal frame
//
// .SECTION See Also
// vtkFrameWidgetRepresentation vtkFrameWidgetRepresentation

#ifndef __vtkFrameWidget_h
#define __vtkFrameWidget_h

#include "vtkAbstractWidget.h"

#include <vtkDRCFiltersModule.h>

class vtkFrameWidgetRepresentation;

class VTKDRCFILTERS_EXPORT vtkFrameWidget : public vtkAbstractWidget
{
public:
  // Description:
  // Instantiate the object.
  static vtkFrameWidget *New();

  // Description:
  // Standard class methods for type information and printing.
  vtkTypeMacro(vtkFrameWidget,vtkAbstractWidget);
  void PrintSelf(ostream& os, vtkIndent indent);


  // Description:
  // Create the default widget representation if one is not set. By default,
  // this is an instance of the vtkFrameWidgetRepresentation class.
  void CreateDefaultRepresentation();

protected:
  vtkFrameWidget();
  ~vtkFrameWidget();

  int WidgetState;
  enum _WidgetState {Start=0,Active};

  // These methods handle events
  static void SelectAction(vtkAbstractWidget*);
  static void EndSelectAction(vtkAbstractWidget*);
  static void MoveAction(vtkAbstractWidget*);

private:
  vtkFrameWidget(const vtkFrameWidget&);  //Not implemented
  void operator=(const vtkFrameWidget&);  //Not implemented
};
 
#endif
