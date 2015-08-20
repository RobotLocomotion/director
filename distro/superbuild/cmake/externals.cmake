

option(USE_PCL "Build with PCL." OFF)
option(USE_LCM "Build with lcm." OFF)
option(USE_LCMGL "Build with lcm-gl." OFF)
option(USE_LIBBOT "Build with libbot." OFF)
option(USE_DRAKE "Build with drake." OFF)
option(USE_STANDALONE_LCMGL "Build with standalone bot-lcmgl." OFF)
set(USE_EIGEN ${USE_PCL})

option(USE_SYSTEM_EIGEN "Use system version of eigen.  If off, eigen will be built." OFF)
option(USE_SYSTEM_LCM "Use system version of lcm.  If off, lcm will be built." OFF)
option(USE_SYSTEM_LIBBOT "Use system version of libbot.  If off, libbot will be built." OFF)

set(default_cmake_args
  "-DCMAKE_PREFIX_PATH:PATH=${install_prefix};${CMAKE_PREFIX_PATH}"
  "-DCMAKE_INSTALL_PREFIX:PATH=${install_prefix}"
  "-DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}"
  "-DBUILD_SHARED_LIBS:BOOL=ON"
  "-DBUILD_DOCUMENTATION:BOOL=OFF"
  "-DENABLE_TESTING:BOOL=OFF"
  )

# Find required external dependencies
find_package(Qt4 4.8 REQUIRED)

set(qt_args
  -DQT_QMAKE_EXECUTABLE:PATH=${QT_QMAKE_EXECUTABLE}
  )

if(APPLE)
  find_program(PYTHON_CONFIG_EXECUTABLE python-config)
  if (NOT PYTHON_CONFIG_EXECUTABLE)
    message(SEND_ERROR "python-config executable not found, but python is required.")
  endif()
  # using "python-config --prefix" so that cmake always uses the python that is
  # in the users path, this is a fix for homebrew on Mac:
  # https://github.com/Homebrew/homebrew/issues/25118
  execute_process(COMMAND ${PYTHON_CONFIG_EXECUTABLE} --prefix OUTPUT_VARIABLE python_prefix OUTPUT_STRIP_TRAILING_WHITESPACE)
  set(PYTHON_INCLUDE_DIR ${python_prefix}/include/python2.7)
  set(PYTHON_LIBRARY ${python_prefix}/lib/libpython2.7${CMAKE_SHARED_LIBRARY_SUFFIX})
else()
  find_package(PythonLibs 2.7 REQUIRED)
endif()

set(python_args
  -DPYTHON_INCLUDE_DIR:PATH=${PYTHON_INCLUDE_DIR}
  -DPYTHON_INCLUDE_DIR2:PATH=${PYTHON_INCLUDE_DIR}
  -DPYTHON_LIBRARY:PATH=${PYTHON_LIBRARY}
  )


###############################################################################
# eigen

if (USE_EIGEN AND NOT USE_SYSTEM_EIGEN)

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

set(eigen_depends eigen)

endif()


###############################################################################
# lcm

if (USE_LCM AND NOT USE_SYSTEM_LCM)

  ExternalProject_Add(
    lcm
    URL http://lcm.googlecode.com/files/lcm-1.0.0.tar.gz
    URL_MD5 69bfbdd9e0d7095c5d7423e71b2fb1a9
    CONFIGURE_COMMAND ${source_prefix}/lcm/configure --prefix=${install_prefix}
  )

  set(lcm_depends lcm)

endif()


###############################################################################
# libbot

if(USE_LIBBOT AND NOT USE_SYSTEM_LIBBOT)

  if(NOT USE_LCM)
    message(SEND_ERROR "Error, USE_LIBBOT is enabled but USE_LCM is OFF.")
  endif()

  ExternalProject_Add(libbot
    GIT_REPOSITORY https://github.com/RobotLocomotion/libbot.git
    GIT_TAG c328b73
    CONFIGURE_COMMAND ""
    INSTALL_COMMAND ""
    BUILD_COMMAND $(MAKE) BUILD_PREFIX=${install_prefix} BUILD_TYPE=${CMAKE_BUILD_TYPE}
    BUILD_IN_SOURCE 1

    DEPENDS
      ${lcm_depends}
    )

  set(libbot_depends libbot)

endif()


if(USE_STANDALONE_LCMGL)

  ExternalProject_Add(bot-lcmgl-download
    GIT_REPOSITORY https://github.com/RobotLocomotion/libbot.git
    GIT_TAG c328b73
    SOURCE_DIR ${PROJECT_BINARY_DIR}/src/bot-lcmgl
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    )

  ExternalProject_Add(bot-lcmgl
    SOURCE_DIR ${PROJECT_BINARY_DIR}/src/bot-lcmgl/bot2-lcmgl
    DOWNLOAD_COMMAND ""
    UPDATE_COMMAND ""
    DEPENDS bot-lcmgl-download
    CMAKE_CACHE_ARGS
      ${default_cmake_args}
      -DUSE_BOT_VIS:BOOL=OFF
    )

  set(lcmgl_depends bot-lcmgl)

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
  GIT_REPOSITORY https://github.com/patmarion/ctkPythonConsole
  GIT_TAG 15988c5
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
  GIT_TAG baf10af
  CMAKE_CACHE_ARGS
    ${default_cmake_args}
    ${qt_args}
  )


###############################################################################
# vtk
set(use_system_vtk_default ON)
option(USE_SYSTEM_VTK "Use system version of VTK.  If off, VTK will be built." ${use_system_vtk_default})

if(NOT USE_SYSTEM_VTK)
  ExternalProject_Add(vtk
    GIT_REPOSITORY https://github.com/bilke/VTK.git
    GIT_TAG b753b7f # vtk 5.10 with fixes for Visual Studio 2013
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
else()

  set(vtk_homebrew_dir /usr/local/opt/vtk5/lib/vtk-5.10)
  if (APPLE AND IS_DIRECTORY ${vtk_homebrew_dir})
    set(vtk_args -DVTK_DIR:PATH=${vtk_homebrew_dir})
  endif()

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
      ${eigen_depends}
      flann
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

    -DUSE_LCM:BOOL=${USE_LCM}
    -DUSE_LCMGL:BOOL=${USE_LCMGL}
    -DUSE_LIBBOT:BOOL=${USE_LIBBOT}
    -DUSE_DRAKE:BOOL=${USE_DRAKE}

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
    ${eigen_depends}
    ${lcm_depends}
    ${libbot_depends}
    PythonQt
    ctkPythonConsole
    QtPropertyBrowser

  )
