#ifndef __QVTKOpenGLInit_h
#define __QVTKOpenGLInit_h

/*
 * Initialization class that creates a valid QSurfaceFormat i.e. OpenGL context
 * with the expected parameters for the QVTKOpenGLWidget.
 * Typical use case is to construct the class before constructing a
 * QApplication object in the main function.
 *
 * Typical usage for QVTKOpenGLInit is as follows:
 * @code{.cpp}
 *
 *   int main(int argc, char* argv[])
 *   {
 *     // Initialize before constructing the QApplication
 *     QVTKOpenGLInit init;
 *     // Construct the QApplication
 *     QApplication app(argc, argv);
 *     // Show the application (that uses QVTKOpenGLWidget)
 *     app.exec();
 *     return 0;
 *   }
 *
 * @endcode
 */

#include "ddAppConfigure.h"

class DD_APP_EXPORT QVTKOpenGLInit
{
public:
  QVTKOpenGLInit();
};

#endif //__QVTKOpenGLInit_h
