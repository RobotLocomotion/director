setup_qt4()
use_cpp11()

find_package(PythonLibs REQUIRED)
find_package(PythonInterp REQUIRED)
find_package(PythonQt REQUIRED)
find_package(CTKPythonConsole REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Drake REQUIRED)
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread filesystem regex)
find_package(VTK REQUIRED)
find_package(QtPropertyBrowser REQUIRED)
find_package(LCM REQUIRED)

include(${VTK_USE_FILE})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_SOURCE_DIR}/src/common)
include_directories(${QTPROPERTYBROWSER_INCLUDE_DIRS})
include_directories(${CTK_PYTHONCONSOLE_INCLUDE_DIRS})
include_directories(${PYTHONQT_INCLUDE_DIRS})
include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${DRAKE_INCLUDE_DIRS})
include_directories(${LCM_INCLUDE_DIRS})

set(decorator_file ${CMAKE_CURRENT_BINARY_DIR}/ddPythonQtDecorators.h)

qt4_wrap_cpp(moc_srcs
  ${decorator_file}
  ddBotImageQueue.h
  ddDrakeModel.h
  ddDRCView.h
  ddLCMSubscriber.h
  ddLCMThread.h
  ddLumberSelection.h
  ddGLWidgetView.h
  ddMacrosManager.h
  ddMainWindow.h
  ddObjectTree.h
  ddPropertiesPanel.h
  ddPythonEventFilter.h
  ddPythonManager.h
  ddQVTKWidgetView.h
  ddSignalMap.h
  ddSpreadsheetView.h
  ddTaskSelection.h
  ddViewBase.h
  ddViewManager.h
  ddViewMenu.h
  )

qt4_wrap_ui(ui_srcs
  ddMainWindow.ui
  ddLumberSelection.ui
  ddTaskSelection.ui
  )

qt4_add_resources(resource_srcs
  uiresources.qrc
  images/resources.qrc
  )

set(srcs
  ${moc_srcs}
  ${ui_srcs}
  ${resource_srcs}

  ddBotImageQueue.cpp
  ddDrakeModel.cpp
  ddDRCView.cpp
  ddLumberSelection.cpp
  ddLCMThread.cpp
  ddGLWidgetView.cpp
  ddMacrosManager.cpp
  ddMainWindow.cpp
  ddObjectTree.cpp
  ddPropertiesPanel.cpp
  ddPythonManager.cpp
  ddPythonQtWrapperFactory.cpp
  ddQVTKWidgetView.cpp
  ddSignalMap.cpp
  ddSpreadsheetView.cpp
  ddTaskSelection.cpp
  ddViewBase.cpp
  ddViewManager.cpp
  ddViewMenu.cpp

  vtkSimpleActorInteractor.cxx
  vtkTDxInteractorStyleCallback.cxx
  )

set(VTK_LIBRARIES
  QVTK
  vtkPythonCore
  )

set(deps
  ${QT_LIBRARIES}
  ${CTK_PYTHONCONSOLE_LIBRARIES}
  ${PYTHONQT_LIBRARIES}
  ${QTPROPERTYBROWSER_LIBRARY}
  ${OPENGL_LIBRARIES}
  ${VTK_LIBRARIES}
  ${DRAKE_LIBRARIES}
  ${Boost_LIBRARIES}
  ${LCM_LIBRARIES}
  )


add_library(ddApp ${srcs})
target_link_libraries(ddApp ${deps})
use_pkg(ddApp ddApp_pkg_dep bot2-core bot2-frames bot2-param-client image-utils)

add_executable(drake-designer main.cpp)
target_link_libraries(drake-designer ddApp)

add_executable(ddConsoleApp consoleApp.cpp)
target_link_libraries(ddConsoleApp ddApp)

install(TARGETS drake-designer ddConsoleApp ddApp
    RUNTIME DESTINATION ${DD_INSTALL_BIN_DIR}
    LIBRARY DESTINATION ${DD_INSTALL_LIB_DIR}
    ARCHIVE DESTINATION ${DD_INSTALL_LIB_DIR})

add_custom_command(OUTPUT ${decorator_file}
  COMMAND ${PYTHON_EXECUTABLE} WrapPythonQt.py
  -i wrapped_methods.txt -o "${decorator_file}"
  --class-prefixes vtk dd QVTK --qt-class-prefixes dd QVTK
  --module-name dd
  DEPENDS wrapped_methods.txt WrapPythonQt.py
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
