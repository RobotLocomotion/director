#ifndef __ddOpenGL_h
#define __ddOpenGL_h


#ifdef __APPLE__
  #include <OpenGL/gl.h>
  #include <OpenGL/glu.h>
  #include <OpenGL/glext.h>
#else
  #define GL_GLEXT_PROTOTYPES
  #ifdef _WIN32
    #define NOMINMAX
    #include <windows.h>
    #include <GL/gl.h>
    #include <GL/glu.h>
  #else
    #include <GL/gl.h>
    #include <GL/glu.h>
    #include <GL/glext.h>
  #endif
#endif


#define _print_gl_error(os, glEnum) \
{ \
  os << "ERROR: Occured in " << __FILE__ << " at line " << __LINE__ << " (error code: " << #glEnum << ")\n"; \
}


#define checkGLError(os) \
{ \
  GLenum error = glGetError(); \
  if (error != GL_NO_ERROR) \
  { \
    switch(error) \
    { \
      case GL_INVALID_ENUM: { _print_gl_error(os, GL_INVALID_ENUM); break; } \
      case GL_INVALID_FRAMEBUFFER_OPERATION: { _print_gl_error(os, GL_INVALID_FRAMEBUFFER_OPERATION); break; } \
      case GL_INVALID_VALUE: { _print_gl_error(os, GL_INVALID_VALUE); break; } \
      case GL_INVALID_OPERATION: { _print_gl_error(os, GL_INVALID_OPERATION); break; } \
      case GL_OUT_OF_MEMORY: { _print_gl_error(os, GL_OUT_OF_MEMORY); break; } \
    } \
  } \
}



#endif
