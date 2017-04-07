setup_qt4()

if(USE_DRC OR USE_DRAKE)
  use_cpp11()
endif()

find_package(PythonLibs 2.7 REQUIRED)
find_package(PythonInterp 2.7 REQUIRED)
find_package(PythonQt REQUIRED)
find_package(CTKPythonConsole REQUIRED)
find_package(OpenGL REQUIRED)
find_package(VTK 7.1 REQUIRED)
find_package(QtPropertyBrowser REQUIRED)

include(${VTK_USE_FILE})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_SOURCE_DIR}/src/common)
include_directories(${QTPROPERTYBROWSER_INCLUDE_DIRS})
include_directories(${CTK_PYTHONCONSOLE_INCLUDE_DIRS})
include_directories(${PYTHONQT_INCLUDE_DIRS})
include_directories(${PYTHON_INCLUDE_DIRS})

set(decorator_file ${CMAKE_CURRENT_BINARY_DIR}/ddPythonQtDecorators.h)

set(wrap_files
  wrapped_methods.txt
  )

qt5_wrap_cpp(moc_srcs
  ${decorator_file}
  ddLumberSelection.h
  ddGLWidgetView.h
  ddGroupBoxHider.h
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

qt5_wrap_ui(ui_srcs
  ddMainWindow.ui
  ddLumberSelection.ui
  ddTaskSelection.ui
  )

qt5_add_resources(resource_srcs
  uiresources.qrc
  images/resources.qrc
  )


set(srcs
  ${moc_srcs}
  ${ui_srcs}
  ${resource_srcs}

  ddLumberSelection.cpp
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
  vtkWrappingPythonCore
  vtkGUISupportQt
  vtkInteractionWidgets
  vtkRenderingAnnotation
  vtkRenderingGL2PSOpenGL2
  vtkRenderingVolumeOpenGL2
  vtkRenderingFreeType
  vtkInteractionStyle
  vtkIOXML
  vtkIOImage
  vtkIOGeometry
  )

set(deps
  ${QT_LIBRARIES}
  ${CTK_PYTHONCONSOLE_LIBRARIES}
  ${PYTHONQT_LIBRARIES}
  ${QTPROPERTYBROWSER_LIBRARY}
  ${OPENGL_LIBRARIES}
  ${VTK_LIBRARIES}
  )

set(pkg_deps)




#####

if (USE_DRAKE)

  find_package(Drake REQUIRED)
  find_package(Eigen REQUIRED)

  include_directories(SYSTEM
    ${EIGEN_INCLUDE_DIRS}
    ${DRAKE_INCLUDE_DIRS}
  )

  set(moc_srcs)
  qt5_wrap_cpp(moc_srcs
    ddDrakeModel.h
    ddDrakeWrapper.h
  )

  list(APPEND srcs
    ${moc_srcs}
    ddDrakeWrapper.cpp
  )

  # openhumanoids uses an old drake fork which has a divergent api so we
  # use a fork of ddDrakeModel.cpp to encapsulate the api incompatibilities
  if (USE_DRC)
    set(DRAKE_OH_FORK TRUE)
    list(APPEND srcs ddDrakeModelOH.cpp)
  else()
    set(DRAKE_OH_FORK FALSE)
    list(APPEND srcs ddDrakeModel.cpp)
  endif()

  configure_file(ddDrakeVersion.h.in ddDrakeVersion.h)

  list(APPEND wrap_files
    wrapped_methods_drake.txt
  )

  list(APPEND deps
    ${DRAKE_LIBRARIES}
  )

endif()

if (USE_LCM)

  find_package(LCM REQUIRED)
  include_directories(${LCM_INCLUDE_DIRS})

  set(moc_srcs)
  qt5_wrap_cpp(moc_srcs
    ddLCMSubscriber.h
    ddLCMThread.h
  )

  list(APPEND srcs
    ${moc_srcs}
    ddLCMThread.cpp
  )

  list(APPEND deps
    ${LCM_LIBRARIES}
  )

  list(APPEND wrap_files
    wrapped_methods_lcm.txt
  )

endif()

if (USE_DRC)

  find_package(LibBot REQUIRED)
  include_directories(${LIBBOT_INCLUDE_DIRS})

  find_package(PCL REQUIRED)
  include_directories(${PCL_INCLUDE_DIRS})

  set(moc_srcs)
  qt5_wrap_cpp(moc_srcs
    ddBotImageQueue.h
    ddKinectLCM.h
    ddPointCloudLCM.h
  )

  list(APPEND srcs
    ${moc_srcs}
    ddBotImageQueue.cpp
    ddKinectLCM.cpp
    ddPointCloudLCM.cpp
  )

  list(APPEND wrap_files
    wrapped_methods_drc.txt
  )

  list (APPEND deps
    ${LIBBOT_LIBRARIES}
    ${PCL_LIBRARIES}
  )

  list(APPEND pkg_deps
    image-utils multisense_utils kinect-utils lcmtypes_kinect lcmtypes_bot2-core
  )

endif()

#####



add_library(ddApp ${srcs})
target_link_libraries(ddApp ${deps})

if(pkg_deps)
  use_pkg(ddApp ${pkg_deps})
endif()

set(_install_targets ddApp directorPython drake-visualizer)

add_executable(drake-visualizer drakeVisualizerApp.cpp)
target_link_libraries(drake-visualizer ddApp)

if(USE_DRC)
  add_executable(director main.cpp)
  target_link_libraries(director ddApp)
  list(APPEND _install_targets director)
endif()

add_executable(directorPython consoleApp.cpp)
target_link_libraries(directorPython ddApp ${PYTHON_LIBRARIES})

install(TARGETS ${_install_targets}
    RUNTIME DESTINATION ${DD_INSTALL_BIN_DIR}
    LIBRARY DESTINATION ${DD_INSTALL_LIB_DIR}
    ARCHIVE DESTINATION ${DD_INSTALL_LIB_DIR})

if(APPLE)
  install(DIRECTORY drake-visualizer.app
      DESTINATION ${DD_INSTALL_BIN_DIR}
      USE_SOURCE_PERMISSIONS)
endif()

add_custom_command(OUTPUT ${decorator_file}
  COMMAND ${PYTHON_EXECUTABLE} WrapPythonQt.py
  -i ${wrap_files} -o "${decorator_file}"
  --class-prefixes vtk dd QVTK --qt-class-prefixes dd QVTK
  --export-header ddAppConfigure.h --export-symbol DD_APP_EXPORT
  --module-name dd
  DEPENDS ${wrap_files} WrapPythonQt.py
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
