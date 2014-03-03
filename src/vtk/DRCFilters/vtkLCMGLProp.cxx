/*=========================================================================

Program:   Visualization Toolkit

Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
All rights reserved.
See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

This software is distributed WITHOUT ANY WARRANTY; without even
the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkLCMGLProp.h"

#include <vtkObjectFactory.h>
#include <vtkSmartPointer.h>
#include <vtkActor.h>

#include <vtkOpenGL.h>

#include <lcmtypes/bot_lcmgl/data_t.hpp>
#include <bot_lcmgl_render/lcmgl_decode.h>


//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkLCMGLProp);


class vtkLCMGLProp::vtkInternal {
public:
  vtkInternal()
    {
      this->GLData.datalen = 0;
    }

  bot_lcmgl::data_t GLData;

  std::vector<vtkSmartPointer<vtkActor> > Actors;
};

//----------------------------------------------------------------------------
vtkLCMGLProp::vtkLCMGLProp()
{
  this->Internal = new vtkInternal;
}

//----------------------------------------------------------------------------
vtkLCMGLProp::~vtkLCMGLProp()
{
  delete this->Internal;
}

//----------------------------------------------------------------------------
double* vtkLCMGLProp::GetBounds()
{
  static double bounds[6] = {-5, 5, -5, 5, -5, 5};
  return bounds;
}

//----------------------------------------------------------------------------
void vtkLCMGLProp::UpdateGLData(const char* messageData)
{
  int status = this->Internal->GLData.decode(messageData, 0, 1e6);
  if (!status)
    {
    this->Internal->GLData.name = std::string();
    this->Internal->GLData.datalen = 0;
    }
}

//----------------------------------------------------------------------------
void vtkLCMGLProp::ReleaseGraphicsResources(vtkWindow *w)
{
  for (size_t i = 0; i < this->Internal->Actors.size(); ++i)
    {
    this->Internal->Actors[i]->ReleaseGraphicsResources(w);
    }
}

//----------------------------------------------------------------------------
int vtkLCMGLProp::RenderOpaqueGeometry(vtkViewport *v)
{
  if (this->Internal->GLData.datalen)
    {
    glPushMatrix();
    glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_LIGHTING);

    bot_lcmgl_decode(&this->Internal->GLData.data.front(), this->Internal->GLData.datalen);

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
int vtkLCMGLProp::RenderOverlay(vtkViewport *v)
{
  int count=0;

  for (size_t i = 0; i < this->Internal->Actors.size(); ++i)
    {
    count += this->Internal->Actors[i]->RenderOverlay(v);
    }

  return count;
}

//----------------------------------------------------------------------------
int vtkLCMGLProp::RenderTranslucentPolygonalGeometry(vtkViewport *v)
{
  int count=0;

  for (size_t i = 0; i < this->Internal->Actors.size(); ++i)
    {
    count += this->Internal->Actors[i]->RenderTranslucentPolygonalGeometry(v);
    }

  return count;
}

//----------------------------------------------------------------------------
int vtkLCMGLProp::HasTranslucentPolygonalGeometry()
{
  int result = 0;

  for (size_t i = 0; i < this->Internal->Actors.size(); ++i)
    {
    result |= this->Internal->Actors[i]->HasTranslucentPolygonalGeometry();
    }

  return result;
}

//----------------------------------------------------------------------------
void vtkLCMGLProp::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
}
