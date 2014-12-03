/*=========================================================================

   Program: ParaView
   Module:    vtkEDLShading.h

   Copyright (c) 2005-2008 Sandia Corporation, Kitware Inc.
   All rights reserved.

   ParaView is a free software; you can redistribute it and/or modify it
   under the terms of the ParaView license version 1.2.

   See License_v1.2.txt for the full ParaView license.
   A copy of this license can be obtained by contacting
   Kitware Inc.
   28 Corporate Drive
   Clifton Park, NY 12065
   USA

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

=========================================================================*/
/*----------------------------------------------------------------------
Acknowledgement:
This algorithm is the result of joint work by Electricité de France,
CNRS, Collège de France and Université J. Fourier as part of the
Ph.D. thesis of Christian BOUCHENY.
------------------------------------------------------------------------*/
// .NAME vtkEDLShading
//
// .SECTION Description
// Implement an EDL offscreen shading.
// Shade the image renderered by its delegate. Two image resolutions are used
//
// This pass expects an initialized depth buffer and color buffer.
// Initialized buffers means they have been cleared with farest z-value and
// background color/gradient/transparent color.
// An opaque pass may have been performed right after the initialization.
//
// The delegate is used once.
//
// Its delegate is usually set to a vtkCameraPass or to a post-processing pass.
//
// This pass requires a OpenGL context that supports texture objects (TO),
// framebuffer objects (FBO) and GLSL. If not, it will emit an error message
// and will render its delegate and return.
//

#ifndef __vtkEDLShading_h
#define __vtkEDLShading_h

#define EDL_HIGH_RESOLUTION_ON 1
#define EDL_LOW_RESOLUTION_ON 1

//#define VTK_EDL_SHADING_DEBUG

#include "vtkDepthImageProcessingPass.h"

class vtkOpenGLRenderWindow;
class vtkDepthPeelingPassLayerList; // Pimpl
class vtkShaderProgram2;
class vtkShader2;
class vtkFrameBufferObject;
class vtkTextureObject;

class VTK_EXPORT vtkEDLShading : public vtkDepthImageProcessingPass
{
public:
  static vtkEDLShading *New();
  vtkTypeRevisionMacro(vtkEDLShading,vtkDepthImageProcessingPass);
  void PrintSelf(ostream& os, vtkIndent indent);

  //BTX
  // Description:
  // Perform rendering according to a render state \p s.
  // \pre s_exists: s!=0
  virtual void Render(const vtkRenderState *s);
  //ETX

  // Description:
  // Release graphics resources and ask components to release their own
  // resources.
  // \pre w_exists: w!=0
  void ReleaseGraphicsResources(vtkWindow *w);

 protected:
  // Description:
  // Default constructor. DelegatePass is set to NULL.
  vtkEDLShading();

  // Description:
  // Destructor.
  virtual ~vtkEDLShading();

  // Description:
  // Initialization of required framebuffer objects
  void EDLInitializeFramebuffers(vtkRenderState &s);

  // Description:
  // Initialization of required GLSL shaders
  void EDLInitializeShaders();

  // Description:
  // Render EDL in full resolution buffer
  bool EDLShadeHigh(vtkRenderState &s);

  // Description:
  // Render EDL in middle resolution buffer
  bool EDLShadeLow(vtkRenderState &s);

  // Description:
  // Render EDL in middle resolution buffer
  bool EDLBlurLow(vtkRenderState &s);

  // Description:
  // Compose color and shaded images
  bool EDLCompose(const vtkRenderState *s);

  // Description:
  // Framebuffer object and textures for initial projection
  vtkFrameBufferObject  *ProjectionFBO;
                        // used to record scene data
  vtkTextureObject      *ProjectionColorTexture;
                        // color render target for projection pass
  vtkTextureObject      *ProjectionDepthTexture;
                        // depth render target for projection pass

  // Framebuffer objects and textures for EDL
  vtkFrameBufferObject *EDLHighFBO;
                       // for EDL full res shading
  vtkTextureObject     *EDLHighShadeTexture;
                       // color render target for EDL full res pass
  vtkFrameBufferObject *EDLLowFBO;
                       // for EDL low res shading (image size/4)
  vtkTextureObject     *EDLLowShadeTexture;
                       // color render target for EDL low res pass
  vtkTextureObject     *EDLLowBlurTexture;
                       // color render target for EDL low res
                       // bilateral filter pass

  // Shader prohrams
  vtkShaderProgram2 *EDLShadeProgram;
  vtkShaderProgram2 *EDLComposeProgram;
  vtkShaderProgram2 *BilateralProgram;

  float EDLNeighbours[32];
  bool  EDLIsFiltered;
  int   EDLLowResFactor; // basically 4

  float Zn;  // near clipping plane
  float Zf;  // far clipping plane

 private:
  vtkEDLShading(const vtkEDLShading&);  // Not implemented.
  void operator=(const vtkEDLShading&);  // Not implemented.
};

#endif
