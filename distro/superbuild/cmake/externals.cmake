

option(USE_PCL "Build with PCL." OFF)
option(USE_LCM "Build with lcm." OFF)
option(USE_LCMGL "Build with lcm-gl." OFF)
option(USE_OCTOMAP "Build with octomap." OFF)
option(USE_APRILTAGS "Build with apriltags lcm driver." OFF)
option(USE_KINECT "Build with kinect lcm driver." OFF)
option(USE_COLLECTIONS "Build with collections." OFF)
option(USE_LIBBOT "Build with libbot." OFF)
option(USE_DRAKE "Build with drake." OFF)
option(USE_STANDALONE_LCMGL "Build with standalone bot-lcmgl." OFF)

option(USE_SYSTEM_EIGEN "Use system version of eigen.  If off, eigen will be built." OFF)
option(USE_SYSTEM_LCM "Use system version of lcm.  If off, lcm will be built." OFF)
option(USE_SYSTEM_LIBBOT "Use system version of libbot.  If off, libbot will be built." OFF)
option(USE_SYSTEM_PCL "Use system version of pcl.  If off, pcl will be built." OFF)

if(USE_DRAKE)
  set(DRAKE_SOURCE_DIR CACHE PATH "")
  set(DRAKE_SUPERBUILD_PREFIX_PATH "")
  if(DRAKE_SOURCE_DIR)
    set(DRAKE_SUPERBUILD_PREFIX_PATH "${DRAKE_SOURCE_DIR}/build/install")
    if(NOT EXISTS "${DRAKE_SUPERBUILD_PREFIX_PATH}")
        message(SEND_ERROR "Cannot find build directory in DRAKE_SOURCE_DIR: ${DRAKE_SOURCE_DIR}")
    endif()
  endif()
endif()

set(default_cmake_args
  "-DCMAKE_PREFIX_PATH:PATH=${install_prefix};${DRAKE_SUPERBUILD_PREFIX_PATH};${CMAKE_PREFIX_PATH}"
  "-DCMAKE_INSTALL_PREFIX:PATH=${install_prefix}"
  "-DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}"
  "-DBUILD_SHARED_LIBS:BOOL=ON"
  "-DBUILD_DOCUMENTATION:BOOL=OFF"
  "-DENABLE_TESTING:BOOL=OFF"
  )

# Find required external dependencies
find_package(Qt5 REQUIRED Core Gui Widgets OpenGL)

set(qt_args
  -DQt5_DIR:PATH=${Qt5_DIR}
  -DQt5Core_DIR:PATH=${Qt5Core_DIR}
  -DQt5Gui_DIR:PATH=${Qt5Gui_DIR}
  -DQt5Widgets_DIR:PATH=${Qt5Widgets_DIR}
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

if (NOT USE_SYSTEM_EIGEN)

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


  if(CMAKE_VERSION VERSION_LESS 3.1)
    ExternalProject_Add(
      cmake3
      URL https://cmake.org/files/v3.5/cmake-3.5.2-Linux-x86_64.tar.gz
      URL_MD5 c7a119aad057a3c0508a2c6d281c6291
      CONFIGURE_COMMAND ""
      BUILD_COMMAND ""
      INSTALL_COMMAND ""
    )
    set(cmake3_args CMAKE_COMMAND ${PROJECT_BINARY_DIR}/src/cmake3/bin/cmake)
    set(cmake3_depends cmake3)
  endif()

  ExternalProject_Add(lcm
    GIT_REPOSITORY https://github.com/lcm-proj/lcm.git
    GIT_TAG 89f26a4
    ${cmake3_args}
    CMAKE_CACHE_ARGS
      ${default_cmake_args}
      ${python_args}
    DEPENDS
      ${cmake3_depends}
    )

  set(lcm_depends lcm ${cmake3_depends})
endif()


###############################################################################
# libbot

if(USE_LIBBOT AND NOT USE_SYSTEM_LIBBOT)

  if(NOT USE_LCM)
    message(SEND_ERROR "Error, USE_LIBBOT is enabled but USE_LCM is OFF.")
  endif()

  ExternalProject_Add(libbot
    GIT_REPOSITORY https://github.com/RobotLocomotion/libbot2.git
    GIT_TAG bc73f60
    CMAKE_CACHE_ARGS
      ${default_cmake_args}
      -DWITH_BOT_VIS:BOOL=OFF

    DEPENDS
      ${lcm_depends}
    )

  set(libbot_depends libbot)

endif()


###############################################################################
# lcm message types repos

if (USE_LCM AND NOT USE_SYSTEM_LIBBOT)

  ExternalProject_Add(bot_core_lcmtypes
    GIT_REPOSITORY https://github.com/openhumanoids/bot_core_lcmtypes
    GIT_TAG 9967654
    ${cmake3_args}
    CMAKE_CACHE_ARGS
      ${default_cmake_args}
      ${python_args}

    DEPENDS
      ${lcm_depends}

      # build bot_core_lcmtypes after libbot, even though it is not a dependency.
      # see https://github.com/RobotLocomotion/libbot/issues/20
      ${libbot_depends}
    )

  ExternalProject_Add(robotlocomotion-lcmtypes
    GIT_REPOSITORY https://github.com/robotlocomotion/lcmtypes
    GIT_TAG 4bd59a1
    ${cmake3_args}
    CMAKE_CACHE_ARGS
      ${default_cmake_args}
      ${python_args}

    DEPENDS
      ${lcm_depends}
      bot_core_lcmtypes
    )

    list(APPEND lcm_depends bot_core_lcmtypes robotlocomotion-lcmtypes)

endif()


###############################################################################
if(USE_STANDALONE_LCMGL)

  if(USE_LIBBOT)
    message(SEND_ERROR "USE_LIBBOT and USE_STANDALONE_LCMGL are incompatible.  Please disable one options.")
  endif()

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
  GIT_TAG patched-7
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
#  GIT_REPOSITORY https://github.com/patmarion/ctkPythonConsole
#  GIT_TAG 15988c5
  GIT_REPOSITORY https://github.com/mwoehlke-kitware/ctkPythonConsole
  GIT_TAG qt5
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
#  GIT_REPOSITORY https://github.com/patmarion/QtPropertyBrowser
#  GIT_TAG baf10af
  GIT_REPOSITORY https://github.com/intbots/QtPropertyBrowser
  GIT_TAG master
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
    GIT_REPOSITORY git://vtk.org/VTK.git
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
      -DCMAKE_CXX_FLAGS:STRING=-DGLX_GLXEXT_LEGACY # fixes compile error on ubuntu 16.04
    )

  set(vtk_args -DVTK_DIR:PATH=${install_prefix}/lib/vtk-5.10)
  set(vtk_depends vtk)
else()

#  set(vtk_homebrew_dir /usr/local/opt/vtk5/lib/vtk-5.10)
#  if (APPLE AND IS_DIRECTORY ${vtk_homebrew_dir})
#    set(vtk_args -DVTK_DIR:PATH=${vtk_homebrew_dir})
#  endif()

  # Verifies that the system has VTK5.
  find_package(VTK REQUIRED HINTS ${vtk_homebrew_dir})
  if (NOT ${VTK_VERSION_MAJOR} EQUAL 7)
    message(FATAL_ERROR "System does not have VTK 7. It has version ${VTK_VERSION}.")
  endif()

  set(vtk_args -DVTK_DIR:PATH=${VTK_DIR})
endif()



###############################################################################
# pcl, flann

if(USE_PCL AND NOT USE_SYSTEM_PCL)

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
    GIT_TAG pcl-1.8.0
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
      -DCMAKE_CXX_FLAGS:STRING=-std=c++11

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
  GIT_TAG cb119b0
  CMAKE_CACHE_ARGS
    ${default_cmake_args}
    ${eigen_args}
    ${boost_args}
    ${flann_args}
    ${vtk_args}
    -DPCL_REQUIRED_VERSION:STRING=1.8.0
  DEPENDS
    ${pcl_depends}
    ${vtk_depends}
  )

endif()


###############################################################################
# camera driver

if(USE_KINECT)

  ExternalProject_Add(openni2-camera-lcm
    GIT_REPOSITORY https://github.com/openhumanoids/openni2-camera-lcm
    GIT_TAG 6b1fe91
    CMAKE_CACHE_ARGS
      ${default_cmake_args}
      -DINSTALL_BOT_SPY:BOOL=OFF
    DEPENDS
      ${lcm_depends}
    )


  ExternalProject_Add(cv-utils
    GIT_REPOSITORY https://github.com/patmarion/cv-utils
    GIT_TAG 1d1465d
    CMAKE_CACHE_ARGS
      ${default_cmake_args}
    DEPENDS
      ${lcm_depends} ${pcl_depends}
    )

  #ExternalProject_Add(kinect
  #  GIT_REPOSITORY https://github.com/openhumanoids/kinect.git
  #  GIT_TAG 3e94f58
  #  CMAKE_CACHE_ARGS
  #    ${default_cmake_args}
  #  DEPENDS
  #    ${lcm_depends} ${libbot_depends}
  #  )

  set(cvutils_depends cv-utils)

endif()


###############################################################################
# apriltags

if(USE_APRILTAGS)

  ExternalProject_Add(apriltags
    GIT_REPOSITORY https://github.com/psiorx/apriltags-pod.git
    GIT_TAG ed2972f
    CMAKE_CACHE_ARGS
      ${default_cmake_args}
    )

  ExternalProject_Add(apriltags_driver
    GIT_REPOSITORY https://github.com/patmarion/apriltags_driver.git
    GIT_TAG 3d84ae4
    CMAKE_CACHE_ARGS
      ${default_cmake_args}
    DEPENDS
      ${lcm_depends} apriltags
    )

endif()


###############################################################################
# director

ExternalProject_Add(director
  SOURCE_DIR ${Superbuild_SOURCE_DIR}/../..
  DOWNLOAD_COMMAND ""
  CMAKE_CACHE_ARGS

    -DUSE_LCM:BOOL=${USE_LCM}
    -DUSE_LCMGL:BOOL=${USE_LCMGL}
    -DUSE_OCTOMAP:BOOL=${USE_OCTOMAP}
    -DUSE_COLLECTIONS:BOOL=${USE_COLLECTIONS}
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
    ${cvutils_depends}
    PythonQt
    ctkPythonConsole
    QtPropertyBrowser

  )
