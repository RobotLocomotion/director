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
  static double bounds[6] = {-55, 55, -55, 55, -55, 55};
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

    // invoke lcmgl rendering
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
