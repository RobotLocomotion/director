

option(USE_PCL "Build PCL." OFF)
option(USE_LIBBOT "Build libbot." OFF)
option(USE_LCM "Build lcm." OFF)
set(USE_EIGEN ${USE_PCL})


set(default_cmake_args
  "-DCMAKE_PREFIX_PATH:PATH=${install_prefix}"
  "-DCMAKE_INSTALL_PREFIX:PATH=${install_prefix}"
  "-DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}"
  "-DBUILD_SHARED_LIBS:BOOL=ON"
  "-DBUILD_DOCUMENTATION:BOOL=OFF"
  "-DENABLE_TESTING:BOOL=OFF"
  )


# Find required external dependencies
find_package(Qt4 4.8 REQUIRED)
find_package(PythonInterp REQUIRED)
find_package(PythonLibs REQUIRED)

set(qt_args
  -DQT_QMAKE_EXECUTABLE:PATH=${QT_QMAKE_EXECUTABLE}
  )

set(python_args
  -DPYTHON_EXECUTABLE:PATH=${PYTHON_EXECUTABLE}
  -DPYTHON_INCLUDE_DIR:PATH=${PYTHON_INCLUDE_DIR}
  -DPYTHON_LIBRARY:PATH=${PYTHON_LIBRARY}
  )


###############################################################################
# libbot

if(USE_LIBBOT)

  ExternalProject_Add(libbot
    GIT_REPOSITORY https://github.com/RobotLocomotion/libbot.git
    GIT_TAG 5561ed5
    CONFIGURE_COMMAND ""
    INSTALL_COMMAND ""
    BUILD_COMMAND $(MAKE) BUILD_PREFIX=${install_prefix} BUILD_TYPE=${CMAKE_BUILD_TYPE}
    BUILD_IN_SOURCE 1
    )

  ExternalProject_Add_Step(libbot edit_libbot_tobuild
    COMMAND ${CMAKE_COMMAND} -E copy ${Superbuild_SOURCE_DIR}/cmake/libbot_tobuild.txt ${source_prefix}/libbot/tobuild.txt
    DEPENDEES download
    DEPENDERS configure)

  set(libbot_depends libbot)

endif()

###############################################################################
# eigen

if (USE_EIGEN)

ExternalProject_Add(
  eigen
  URL http://www.vtk.org/files/support/eigen-3.2.1.tar.gz
  URL_MD5 a0e0a32d62028218b1c1848ad7121476
  CMAKE_CACHE_ARGS
    ${default_cmake_args}
)

ExternalProject_Add_Step(eigen make_pkgconfig_dir
  COMMAND ${CMAKE_COMMAND} -E make_directory ${install_prefix}/lib/pkgconfig
  DEPENDERS configure)

set(eigen_args
  -DEIGEN_INCLUDE_DIR:PATH=${install_prefix}/include/eigen3
  -DEIGEN_INCLUDE_DIRS:PATH=${install_prefix}/include/eigen3
  -DEIGEN3_INCLUDE_DIR:PATH=${install_prefix}/include/eigen3
  )

endif()

###############################################################################
# lcm

if (USE_LCM)

  ExternalProject_Add(
    lcm
    URL http://lcm.googlecode.com/files/lcm-1.0.0.tar.gz
    URL_MD5 69bfbdd9e0d7095c5d7423e71b2fb1a9
    CONFIGURE_COMMAND ${source_prefix}/lcm/configure --prefix=${install_prefix}
  )

  set(lcm_depends lcm)

endif()


###############################################################################
# PythonQt
ExternalProject_Add(PythonQt
  GIT_REPOSITORY https://github.com/commontk/PythonQt.git
  GIT_TAG 00e6c6b2
  CMAKE_CACHE_ARGS
    ${default_cmake_args}
    ${qt_args}
    ${python_args}
    -DPythonQt_Wrap_Qtcore:BOOL=ON
    -DPythonQt_Wrap_Qtgui:BOOL=ON
    -DPythonQt_Wrap_Qtuitools:BOOL=ON
  )

###############################################################################
# ctkPythonConsole
ExternalProject_Add(ctkPythonConsole
  GIT_REPOSITORY http://github.com/patmarion/ctkPythonConsole
  GIT_TAG b24d917ad
  CMAKE_CACHE_ARGS
    ${default_cmake_args}
    ${qt_args}
    ${python_args}
  DEPENDS
    PythonQt
  )

###############################################################################
# QtPropertyBrowser
ExternalProject_Add(QtPropertyBrowser
  GIT_REPOSITORY https://github.com/patmarion/QtPropertyBrowser
  GIT_TAG origin/master
  CMAKE_CACHE_ARGS
    ${default_cmake_args}
    ${qt_args}
  )


###############################################################################
# vtk
set(use_system_vtk_default ON)
if(APPLE)
  set(use_system_vtk_default OFF)
endif()

option(USE_SYSTEM_VTK "Use system version of VTK.  If off, VTK will be built." ${use_system_vtk_default})

if(NOT USE_SYSTEM_VTK)
  ExternalProject_Add(vtk
    GIT_REPOSITORY http://vtk.org/VTK.git
    GIT_TAG v5.10.1
    CMAKE_CACHE_ARGS
      ${default_cmake_args}
      ${python_args}
      ${qt_args}
      -DBUILD_SHARED_LIBS:BOOL=ON
      -DBUILD_TESTING:BOOL=OFF
      -DBUILD_EXAMPLES:BOOL=OFF
      -DVTK_USE_GUISUPPORT:BOOL=ON
      -DVTK_USE_QT:BOOL=ON
      -DVTK_WRAP_PYTHON:BOOL=ON
      -DVTK_WRAP_TCL:BOOL=OFF
      -DVTK_USE_TK:BOOL=OFF
    )

  set(vtk_args -DVTK_DIR:PATH=${install_prefix}/lib/vtk-5.10)
  set(vtk_depends vtk)
endif()



###############################################################################
# pcl, flann

if(USE_PCL)

  # boost is an external dependency
  find_package(Boost REQUIRED)
  set(boost_args
    -DBoost_INCLUDE_DIR:PATH=${Boost_INCLUDE_DIR}
  )

  ExternalProject_Add(
    flann
    GIT_REPOSITORY http://github.com/mariusmuja/flann
    GIT_TAG cee08ec38a8df7bc70397f10a4d30b9b33518bb4
    CMAKE_CACHE_ARGS
      ${default_cmake_args}
      ${boost_args}
      ${python_args}
      -DBUILD_MATLAB_BINDINGS:BOOL=OFF
      -DBUILD_PYTHON_BINDINGS:BOOL=OFF
      -DBUILD_C_BINDINGS:BOOL=OFF
  )

  # flann used to install to lib64, but it seems that it doesn't do that anymore...
  if(FALSE AND NOT APPLE AND ${CMAKE_SYSTEM_PROCESSOR} STREQUAL x86_64)
    set(flann_lib_dir lib64)
  else()
    set(flann_lib_dir lib)
  endif()

  set(so_extension so)
  if(APPLE)
    set(so_extension dylib)
  endif()

  set(flann_args
    -DFLANN_INCLUDE_DIR:PATH=${install_prefix}/include
    -DFLANN_INCLUDE_DIRS:PATH=${install_prefix}/include
    -DFLANN_LIBRARY:PATH=${install_prefix}/${flann_lib_dir}/libflann_cpp.${so_extension}
    -DFLANN_LIBRARY_DEBUG:PATH=${install_prefix}/${flann_lib_dir}/libflann_cpp-gd.${so_extension}
    )


  ExternalProject_Add(
    pcl
    GIT_REPOSITORY http://github.com/pointcloudlibrary/pcl.git
    GIT_TAG pcl-1.7.1
    CMAKE_CACHE_ARGS
      ${default_cmake_args}
      ${eigen_args}
      ${boost_args}
      ${flann_args}
      ${vtk_args}
      -DPCL_SHARED_LIBS:BOOL=ON
      -DBUILD_TESTS:BOOL=OFF
      -DBUILD_global_tests:BOOL=OFF
      -DBUILD_examples:BOOL=OFF
      -DBUILD_tools:BOOL=OFF
      -DBUILD_apps:BOOL=OFF
      -DBUILD_visualization:BOOL=OFF

    DEPENDS
      ${vtk_depends}
      flann
      eigen
  )

  set(pcl_depends pcl)

endif()


###############################################################################
# PointCloudLibraryPlugin

if(USE_PCL)

ExternalProject_Add(PointCloudLibraryPlugin
  GIT_REPOSITORY https://github.com/patmarion/PointCloudLibraryPlugin.git
  GIT_TAG filters-library
  INSTALL_COMMAND ""
  CMAKE_CACHE_ARGS
    ${default_cmake_args}
    ${eigen_args}
    ${boost_args}
    ${flann_args}
    ${vtk_args}
    -DPCL_REQUIRED_VERSION:STRING=1.7.1
  DEPENDS
    pcl
    ${vtk_depends}
  )

endif()


###############################################################################
# ddapp

ExternalProject_Add(ddapp
  SOURCE_DIR ${Superbuild_SOURCE_DIR}/../..
  DOWNLOAD_COMMAND ""
  CMAKE_CACHE_ARGS

    -DUSE_PORTMIDI:BOOL=OFF
		-DUSE_DRC:BOOL=OFF
		-DUSE_DRC_MAPS:BOOL=OFF
		-DUSE_DRAKE:BOOL=OFF
    -DUSE_LCM_GL:BOOL=OFF

		-DUSE_LIBBOT:BOOL=${USE_LIBBOT}
    -DUSE_LCM:BOOL=${USE_LCM}

    ${default_cmake_args}
    ${eigen_args}
    ${boost_args}
    ${flann_args}
    ${vtk_args}
    ${python_args}
    ${qt_args}

  DEPENDS

    ${vtk_depends}
    ${pcl_depends}
    ${lcm_depends}
    ${libbot_depends}
    PythonQt
    ctkPythonConsole
    QtPropertyBrowser


  )
