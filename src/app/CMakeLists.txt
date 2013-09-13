setup_qt4()

find_package(PythonLibs REQUIRED)
find_package(PythonInterp REQUIRED)
find_package(QtPropertyBrowser REQUIRED)
find_package(PythonQt REQUIRED)
find_package(CTKPythonConsole REQUIRED)
find_package(OpenGL REQUIRED)
find_package(VTK REQUIRED COMPONENTS vtkRenderingOpenGL vtkGUISupportQt vtkInteractionWidgets vtkWrappingPythonCore)

include(${VTK_USE_FILE})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_SOURCE_DIR}/src/common)
include_directories(${CTK_PYTHONCONSOLE_INCLUDE_DIRS})
include_directories(${PYTHONQT_INCLUDE_DIRS})
include_directories(${PYTHON_INCLUDE_DIRS})

set(decorator_file ${CMAKE_CURRENT_BINARY_DIR}/ddPythonQtDecorators.h)

qt4_wrap_cpp(moc_srcs
  ${decorator_file}
  ddGLWidgetView.h
  ddMainWindow.h
  ddPythonManager.h
  ddQVTKWidgetView.h
  ddViewBase.h
  ddViewManager.h
  )

qt4_wrap_ui(ui_srcs
  ddMainWindow.ui
  )

qt4_add_resources(resource_srcs
  images/resources.qrc
  )

set(srcs
  ${moc_srcs}
  ${ui_srcs}
  ${resource_srcs}

  ddGLWidgetView.cpp
  ddPythonManager.cpp
  ddPythonQtWrapperFactory.cpp
  ddQVTKWidgetView.cpp
  ddMainWindow.cpp
  ddViewBase.cpp
  ddViewManager.cpp
  )

set(deps
  ${QT_LIBRARIES}
  ${CTK_PYTHONCONSOLE_LIBRARIES}
  ${PYTHONQT_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${VTK_LIBRARIES}
  )

add_library(ddApp ${srcs})
target_link_libraries(ddApp ${deps})

add_executable(drake-designer main.cpp)
target_link_libraries(drake-designer ddApp)


add_custom_command(OUTPUT ${decorator_file}
  COMMAND ${PYTHON_EXECUTABLE} WrapPythonQt.py
  -i wrapped_methods.txt -o "${decorator_file}"
  --class-prefixes vtk dd --qt-class-prefixes dd
  --module-name dd
  DEPENDS wrapped_methods.txt WrapPythonQt.py
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
